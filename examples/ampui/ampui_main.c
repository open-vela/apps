/****************************************************************************
 * apps/examples/ampui/ampui_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* A home screen and a camera screen, on the panel this core drives directly.
 *
 * Why this replaces ampcam rather than sitting beside it
 * ----------------------------------------------------------------------------
 * ampcam mmaps /dev/fb0 and writes the first of its two buffers, publishing with
 * FBIO_UPDATE only. LVGL's fbdev backend mmaps the same region, renders into
 * whichever buffer is free and follows up with FBIOPAN_DISPLAY. There is no
 * ownership protocol between them - no lock, no shared state - so running both
 * means two writers on the same pixels and neither picture survives. The camera
 * view therefore has to live inside the LVGL application, which is what this is.
 *
 * Nothing is lost by moving it: the frame still arrives as one copy out of the
 * shared carveout, and the boxes are still drawn with direct stores. What is
 * gained is a real back button, hit-tested by the toolkit, instead of a
 * hand-drawn rectangle and hand-written coordinate comparisons.
 *
 * Why the canvas is the size of the frame and not the size of the screen
 * ----------------------------------------------------------------------------
 * read() on /dev/amcam0 returns whole rows, packed, stride = width * 4. With
 * LV_DRAW_BUF_STRIDE_ALIGN at 1, lv_canvas_set_buffer() computes exactly that
 * same stride, so the frame can be read straight into the canvas buffer. No
 * intermediate buffer, no per-row loop, and the copy count is the same as
 * ampcam's.
 *
 * Sizing the canvas to the frame instead of the panel also deletes the centring
 * arithmetic ampcam needed. A 540x960 frame fills the screen and a 512x288 one
 * is centred with a black border, and the difference is lv_obj_center() rather
 * than an x0/y0 offset threaded through the blit and every box. Boxes are then
 * in the canvas's own coordinates, which is the space the producer already
 * publishes them in.
 *
 * Why the frames are pumped from a timer and not read in a loop
 * ----------------------------------------------------------------------------
 * /dev/amcam0 has no poll method, so there is no way to wait on it together with
 * anything else. A blocking read holds still for up to two seconds when the
 * producer is idle, and doing that from the LVGL thread would freeze the button
 * that gets you out of this screen. O_NONBLOCK turns "no new frame" into -EAGAIN,
 * which is a timer tick with nothing to do.
 *
 * Why there is a heartbeat
 * ----------------------------------------------------------------------------
 * Not decoration. The window this core scans out of is Esmart3 on vp3, and it
 * stays in vp3's plane mask, so a Linux modeset walks it and switches it off -
 * see evb7_amp_vop.c. Every register is rewritten on each flip, so the next
 * frame this core publishes brings the window back. That works while something
 * is being redrawn, and the home screen is static: with nothing invalidated
 * there is no flush, no pan, no flip, and the panel would stay dark from the
 * moment Linux starts its side until the first touch.
 *
 * A once-a-second invalidate of one small object is enough, because any flush
 * reaches evb7_amp_vop_flip(). It is drawn as a visible dot rather than an
 * invisible timer so that a stalled UI looks stalled instead of looking like a
 * display fault.
 *
 * Why the labels are in English
 * ----------------------------------------------------------------------------
 * The only fonts built in are Montserrat 20 and 24, which are Latin. Chinese
 * text would render as empty boxes, and shipping a CJK font is several hundred
 * kilobytes of glyph data in an image that is currently under a megabyte. The
 * camera icon is drawn from primitives for the same reason - there is no camera
 * glyph in the built-in symbol set.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <lvgl/lvgl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMPUI_CAM_DEV       "/dev/amcam0"

/* Frame pump interval. Faster than the 30fps the producer manages, so a frame
 * is picked up in the tick after it appears rather than up to a whole tick
 * later. A tick with no frame costs one failed read.
 */

#define AMPUI_FRAME_MS      16

/* Heartbeat, and the interval the on-screen counters are refreshed at. One
 * second is short enough that a modeset cannot leave the panel dark for long,
 * and long enough that the redraw it forces is not worth measuring.
 */

#define AMPUI_BEAT_MS       1000

/* Green for a person, amber for anything else. Class 0 is person in the COCO
 * ordering the model was trained on, and it is the class this is looking for,
 * so it gets the colour that reads as a hit. The same two values ampcam used,
 * so a screenshot of either means the same thing.
 */

#define AMPUI_BOX_PERSON    0xff00ff00u
#define AMPUI_BOX_OTHER     0xffffaa00u

#define AMPUI_BG            0x101418
#define AMPUI_ACCENT        0x2f7fe0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ampui_s
{
  lv_obj_t   *home;
  lv_obj_t   *cam;
  lv_obj_t   *canvas;
  lv_obj_t   *cam_status;
  lv_obj_t   *home_status;
  lv_obj_t   *beat;

  lv_timer_t *frame_timer;
  lv_timer_t *beat_timer;

  int         camfd;

  /* Canvas backing store, frame-sized. Reallocated if the producer changes
   * geometry, which happens when the Linux side is restarted with a different
   * rotation.
   */

  uint8_t    *buf;
  uint32_t    buf_w;
  uint32_t    buf_h;

  uint32_t    frames;
  uint32_t    last_frames;
  uint32_t    boxes;
  bool        beat_on;
  bool        on_cam;

  /* Off the stack: 64 boxes plus a header is most of a kilobyte, and this is
   * filled in from a timer callback running on the main task's stack.
   */

  struct ampcam_detect_s det;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ampui_s g_ui;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ampui_camera_open
 *
 * Description:
 *   Open the frame device if it is not open already.
 *
 *   Retried rather than opened once at startup, because /dev/amcam0 is
 *   registered when the shared-memory endpoint is set up and this application
 *   may well be running before Linux has finished coming up. A failure here is
 *   a state to display, not an error to exit on.
 *
 ****************************************************************************/

static bool ampui_camera_open(struct ampui_s *ui)
{
  if (ui->camfd >= 0)
    {
      return true;
    }

  ui->camfd = open(AMPUI_CAM_DEV, O_RDONLY | O_NONBLOCK);
  return ui->camfd >= 0;
}

/****************************************************************************
 * Name: ampui_canvas_fit
 *
 * Description:
 *   Make the canvas match the producer's geometry, allocating on the first
 *   frame and reallocating if it ever changes.
 *
 *   Returns false when the buffer could not be allocated, in which case the
 *   caller must not read - the canvas still points at the old buffer, or at
 *   none.
 *
 ****************************************************************************/

static bool ampui_canvas_fit(struct ampui_s *ui, uint32_t w, uint32_t h)
{
  uint8_t *buf;

  if (w == 0 || h == 0)
    {
      return false;
    }

  if (ui->buf != NULL && ui->buf_w == w && ui->buf_h == h)
    {
      return true;
    }

  buf = malloc((size_t)w * h * 4);
  if (buf == NULL)
    {
      return false;
    }

  /* Black, so the first partial frame does not appear over whatever the heap
   * happened to contain.
   */

  memset(buf, 0, (size_t)w * h * 4);

  /* The canvas is pointed at the new buffer before the old one is released,
   * so LVGL never holds a pointer into freed memory even briefly.
   */

  lv_canvas_set_buffer(ui->canvas, buf, (int32_t)w, (int32_t)h,
                       LV_COLOR_FORMAT_XRGB8888);
  lv_obj_center(ui->canvas);

  free(ui->buf);
  ui->buf   = buf;
  ui->buf_w = w;
  ui->buf_h = h;

  printf("ampui: canvas is %" PRIu32 "x%" PRIu32 ", %s\n", w, h,
         (w == (uint32_t)LV_HOR_RES && h == (uint32_t)LV_VER_RES) ?
         "full screen" : "centred");
  return true;
}

/****************************************************************************
 * Name: ampui_hline / ampui_vline
 *
 * Description:
 *   Two-pixel lines straight into the canvas buffer, clipped to it.
 *
 *   Written as stores rather than as LVGL objects because there can be 64 boxes
 *   in a set and a set every frame: creating and deleting a few hundred objects
 *   a second to draw eight rectangles' worth of pixels would cost more than the
 *   pixels do. The buffer is ours, XRGB8888 and packed, so a store is well
 *   defined here.
 *
 *   Clipped rather than trusted. The coordinates come from a model running on
 *   the other core; a box a pixel outside the frame is a plausible rounding
 *   result, and writing it unclipped is a stray store past the buffer.
 *
 ****************************************************************************/

static void ampui_hline(struct ampui_s *ui, int x, int y, int len,
                        uint32_t colour)
{
  int t;

  for (t = 0; t < 2; t++, y++)
    {
      uint32_t *row;
      int i;

      if (y < 0 || y >= (int)ui->buf_h)
        {
          continue;
        }

      row = (uint32_t *)(ui->buf + (size_t)y * ui->buf_w * 4);

      for (i = 0; i < len; i++)
        {
          int px = x + i;

          if (px >= 0 && px < (int)ui->buf_w)
            {
              row[px] = colour;
            }
        }
    }
}

static void ampui_vline(struct ampui_s *ui, int x, int y, int len,
                        uint32_t colour)
{
  int i;

  for (i = 0; i < len; i++)
    {
      int py = y + i;
      uint32_t *row;
      int t;

      if (py < 0 || py >= (int)ui->buf_h)
        {
          continue;
        }

      row = (uint32_t *)(ui->buf + (size_t)py * ui->buf_w * 4);

      for (t = 0; t < 2; t++)
        {
          int px = x + t;

          if (px >= 0 && px < (int)ui->buf_w)
            {
              row[px] = colour;
            }
        }
    }
}

/****************************************************************************
 * Name: ampui_draw_boxes
 *
 * Description:
 *   Outline every detection over the frame just read, plus a bar whose length
 *   is the confidence.
 *
 *   Scaled from the detector's coordinate space rather than assumed equal to
 *   the frame's. They are the same today - the producer publishes boxes in the
 *   published frame's space - but the descriptor carries its own width and
 *   height precisely so this does not become an assumption that breaks quietly
 *   the first time either resolution moves.
 *
 ****************************************************************************/

static void ampui_draw_boxes(struct ampui_s *ui)
{
  const struct ampcam_detect_s *det = &ui->det;
  int i;

  if (det->count <= 0 || det->width == 0 || det->height == 0)
    {
      return;
    }

  for (i = 0; i < det->count && i < AMPCAM_MAXBOX; i++)
    {
      const struct ampcam_box_s *b = &det->box[i];
      uint32_t colour;
      int bx;
      int by;
      int bw;
      int bh;
      int bar;

      bx = (int)((uint64_t)b->x * ui->buf_w / det->width);
      by = (int)((uint64_t)b->y * ui->buf_h / det->height);
      bw = (int)((uint64_t)b->w * ui->buf_w / det->width);
      bh = (int)((uint64_t)b->h * ui->buf_h / det->height);

      if (bw <= 0 || bh <= 0)
        {
          continue;
        }

      colour = (b->cls == 0) ? AMPUI_BOX_PERSON : AMPUI_BOX_OTHER;

      ampui_hline(ui, bx, by, bw, colour);
      ampui_hline(ui, bx, by + bh - 2, bw, colour);
      ampui_vline(ui, bx, by, bh, colour);
      ampui_vline(ui, bx + bw - 2, by, bh, colour);

      /* Confidence as a bar just above the box, moved below it when the box is
       * against the top edge so a detection there still shows one.
       */

      bar = (int)((uint32_t)bw * b->score / 100u);
      if (bar > 0)
        {
          ampui_hline(ui, bx, by > 4 ? by - 4 : by + bh, bar, colour);
        }
    }
}

/****************************************************************************
 * Name: ampui_frame_cb
 *
 * Description:
 *   One tick of the camera screen: take a frame if there is one, put the
 *   current boxes on it, and hand it to the toolkit.
 *
 ****************************************************************************/

static void ampui_frame_cb(lv_timer_t *timer)
{
  struct ampui_s *ui = lv_timer_get_user_data(timer);
  struct ampcam_geom_s geom;
  ssize_t n;

  if (!ampui_camera_open(ui))
    {
      return;
    }

  /* Geometry first, because the buffer has to be the right size before the
   * read rather than after it. The ioctl is a few loads from the descriptor.
   */

  if (ioctl(ui->camfd, AMPCAMIOC_GETGEOM, (unsigned long)&geom) < 0)
    {
      return;
    }

  if (!ampui_canvas_fit(ui, geom.width, geom.height))
    {
      return;
    }

  n = read(ui->camfd, (char *)ui->buf, (size_t)ui->buf_w * ui->buf_h * 4);
  if (n <= 0)
    {
      /* -EAGAIN is the normal case: the producer has not published since the
       * last tick. Everything else is left alone deliberately - the frame
       * already on screen stays there, which is what a dropped frame should
       * look like.
       */

      return;
    }

  ui->frames++;

  /* Detections every frame rather than only when a new set arrives. The
   * producer publishes about ten a second against thirty frames here; drawing
   * them only on the frames where a set happened to be new would leave two
   * frames in three without boxes, which reads as flicker rather than as lag.
   */

  if (ioctl(ui->camfd, AMPCAMIOC_GETDET, (unsigned long)&ui->det) < 0)
    {
      ui->det.count = -1;
    }

  if (ui->det.count > 0)
    {
      ui->boxes = (uint32_t)ui->det.count;
      ampui_draw_boxes(ui);
    }
  else
    {
      ui->boxes = 0;
    }

  lv_obj_invalidate(ui->canvas);
}

/****************************************************************************
 * Name: ampui_beat_cb
 *
 * Description:
 *   The once-a-second redraw that keeps the window programmed, and the counters
 *   that make it visible.
 *
 *   See the note at the top of this file: without something invalidating on a
 *   static screen there is no flip, and without a flip a Linux modeset leaves
 *   the panel dark.
 *
 ****************************************************************************/

static void ampui_beat_cb(lv_timer_t *timer)
{
  struct ampui_s *ui = lv_timer_get_user_data(timer);
  char text[64];

  ui->beat_on = !ui->beat_on;
  lv_obj_set_style_bg_opa(ui->beat, ui->beat_on ? LV_OPA_COVER : LV_OPA_20,
                          LV_PART_MAIN);

  if (ui->on_cam)
    {
      uint32_t fps = ui->frames - ui->last_frames;

      ui->last_frames = ui->frames;

      snprintf(text, sizeof(text), "%" PRIu32 " fps   %" PRIu32 " box%s",
               fps, ui->boxes, ui->boxes == 1 ? "" : "es");
      lv_label_set_text(ui->cam_status, text);
    }
  else
    {
      /* The home screen says whether there is anything to show before the
       * button is pressed, because "the camera page is black" and "the other
       * core is not publishing" are the same picture and different problems.
       */

      struct ampcam_geom_s geom;

      if (!ampui_camera_open(ui))
        {
          lv_label_set_text(ui->home_status, "camera device not ready");
        }
      else if (ioctl(ui->camfd, AMPCAMIOC_GETGEOM,
                     (unsigned long)&geom) < 0 || geom.width == 0)
        {
          lv_label_set_text(ui->home_status, "waiting for the host to stream");
        }
      else
        {
          snprintf(text, sizeof(text), "stream ready  %" PRIu32 "x%" PRIu32,
                   geom.width, geom.height);
          lv_label_set_text(ui->home_status, text);
        }
    }
}

/****************************************************************************
 * Name: ampui_show_cam / ampui_show_home
 *
 * Description:
 *   Switch screens, and start or stop the frame pump with them.
 *
 *   Paused rather than left running, because a timer reading the camera while
 *   the home screen is up would copy two megabytes thirty times a second into a
 *   canvas nothing is drawing.
 *
 ****************************************************************************/

static void ampui_show_cam(lv_event_t *e)
{
  struct ampui_s *ui = lv_event_get_user_data(e);

  ui->on_cam      = true;
  ui->last_frames = ui->frames;

  lv_screen_load(ui->cam);
  lv_timer_resume(ui->frame_timer);
}

static void ampui_show_home(lv_event_t *e)
{
  struct ampui_s *ui = lv_event_get_user_data(e);

  ui->on_cam = false;

  lv_timer_pause(ui->frame_timer);
  lv_screen_load(ui->home);
}

/****************************************************************************
 * Name: ampui_icon
 *
 * Description:
 *   A camera, from four rectangles and two circles.
 *
 *   Drawn rather than loaded: there is no camera glyph in the built-in symbol
 *   set, and an image asset would be a generated C array to keep in step with
 *   nothing. Six styled objects are cheaper to read than that, and they scale
 *   with the numbers here rather than with a bitmap.
 *
 ****************************************************************************/

static void ampui_icon(lv_obj_t *parent)
{
  lv_obj_t *body;
  lv_obj_t *bump;
  lv_obj_t *ring;
  lv_obj_t *glass;
  lv_obj_t *flash;

  /* Viewfinder bump first, so the body's rounded corner overlaps it rather
   * than the other way round.
   */

  bump = lv_obj_create(parent);
  lv_obj_remove_flag(bump, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(bump, 52, 22);
  lv_obj_align(bump, LV_ALIGN_TOP_LEFT, 26, 0);
  lv_obj_set_style_radius(bump, 6, LV_PART_MAIN);
  lv_obj_set_style_bg_color(bump, lv_color_hex(0x3a4450), LV_PART_MAIN);
  lv_obj_set_style_border_width(bump, 0, LV_PART_MAIN);

  body = lv_obj_create(parent);
  lv_obj_remove_flag(body, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(body, 168, 116);
  lv_obj_align(body, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_radius(body, 18, LV_PART_MAIN);
  lv_obj_set_style_bg_color(body, lv_color_hex(0x2b333d), LV_PART_MAIN);
  lv_obj_set_style_border_width(body, 0, LV_PART_MAIN);

  ring = lv_obj_create(body);
  lv_obj_remove_flag(ring, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(ring, 74, 74);
  lv_obj_center(ring);
  lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(ring, lv_color_hex(0x151a20), LV_PART_MAIN);
  lv_obj_set_style_border_width(ring, 3, LV_PART_MAIN);
  lv_obj_set_style_border_color(ring, lv_color_hex(0x8fa2b6), LV_PART_MAIN);

  glass = lv_obj_create(ring);
  lv_obj_remove_flag(glass, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(glass, 34, 34);
  lv_obj_center(glass);
  lv_obj_set_style_radius(glass, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(glass, lv_color_hex(AMPUI_ACCENT), LV_PART_MAIN);
  lv_obj_set_style_border_width(glass, 0, LV_PART_MAIN);

  flash = lv_obj_create(body);
  lv_obj_remove_flag(flash, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(flash, 14, 14);
  lv_obj_align(flash, LV_ALIGN_TOP_RIGHT, -14, 14);
  lv_obj_set_style_radius(flash, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(flash, lv_color_hex(0xf0c14b), LV_PART_MAIN);
  lv_obj_set_style_border_width(flash, 0, LV_PART_MAIN);
}

/****************************************************************************
 * Name: ampui_build_home
 ****************************************************************************/

static void ampui_build_home(struct ampui_s *ui)
{
  lv_obj_t *icon;
  lv_obj_t *title;
  lv_obj_t *btn;
  lv_obj_t *label;

  ui->home = lv_obj_create(NULL);
  lv_obj_remove_flag(ui->home, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(ui->home, lv_color_hex(AMPUI_BG), LV_PART_MAIN);

  title = lv_label_create(ui->home);
  lv_label_set_text(title, "AMP Vision");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, LV_PART_MAIN);
  lv_obj_set_style_text_color(title, lv_color_hex(0xf0f4f8), LV_PART_MAIN);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 60);

  /* A plain container for the icon, so the pieces inside it can be positioned
   * against each other rather than against the screen.
   */

  icon = lv_obj_create(ui->home);
  lv_obj_remove_flag(icon, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(icon, 180, 140);
  lv_obj_align(icon, LV_ALIGN_CENTER, 0, -80);
  lv_obj_set_style_bg_opa(icon, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(icon, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(icon, 0, LV_PART_MAIN);
  ampui_icon(icon);

  btn = lv_button_create(ui->home);
  lv_obj_set_size(btn, 300, 76);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 90);
  lv_obj_set_style_radius(btn, 38, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn, lv_color_hex(AMPUI_ACCENT), LV_PART_MAIN);
  lv_obj_add_event_cb(btn, ampui_show_cam, LV_EVENT_CLICKED, ui);

  label = lv_label_create(btn);
  lv_label_set_text(label, LV_SYMBOL_PLAY "  Open Camera");
  lv_obj_set_style_text_font(label, &lv_font_montserrat_24, LV_PART_MAIN);
  lv_obj_center(label);

  ui->home_status = lv_label_create(ui->home);
  lv_label_set_text(ui->home_status, "starting");
  lv_obj_set_style_text_font(ui->home_status, &lv_font_montserrat_20,
                             LV_PART_MAIN);
  lv_obj_set_style_text_color(ui->home_status, lv_color_hex(0x8fa2b6),
                              LV_PART_MAIN);
  lv_obj_align(ui->home_status, LV_ALIGN_BOTTOM_MID, 0, -60);

  /* The heartbeat lives on the home screen because that is the screen that can
   * otherwise sit still. The camera screen invalidates a canvas thirty times a
   * second and needs no help.
   */

  ui->beat = lv_obj_create(ui->home);
  lv_obj_remove_flag(ui->beat, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(ui->beat, 10, 10);
  lv_obj_align(ui->beat, LV_ALIGN_BOTTOM_MID, 0, -30);
  lv_obj_set_style_radius(ui->beat, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(ui->beat, lv_color_hex(0x4caf50), LV_PART_MAIN);
  lv_obj_set_style_border_width(ui->beat, 0, LV_PART_MAIN);
}

/****************************************************************************
 * Name: ampui_build_cam
 ****************************************************************************/

static void ampui_build_cam(struct ampui_s *ui)
{
  lv_obj_t *back;
  lv_obj_t *label;

  ui->cam = lv_obj_create(NULL);
  lv_obj_remove_flag(ui->cam, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(ui->cam, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_pad_all(ui->cam, 0, LV_PART_MAIN);

  /* The canvas has no buffer yet: its size is the producer's to decide, and it
   * is set on the first frame. Until then this screen is black, which is what
   * the status label is for.
   */

  ui->canvas = lv_canvas_create(ui->cam);
  lv_obj_center(ui->canvas);

  back = lv_button_create(ui->cam);
  lv_obj_set_size(back, 132, 60);
  lv_obj_align(back, LV_ALIGN_TOP_LEFT, 16, 16);
  lv_obj_set_style_radius(back, 30, LV_PART_MAIN);
  lv_obj_set_style_bg_color(back, lv_color_hex(0x000000), LV_PART_MAIN);

  /* Translucent rather than solid: it sits over the picture, and the pixels it
   * covers are the ones a viewer is most likely to want to see past.
   */

  lv_obj_set_style_bg_opa(back, LV_OPA_60, LV_PART_MAIN);
  lv_obj_add_event_cb(back, ampui_show_home, LV_EVENT_CLICKED, ui);

  label = lv_label_create(back);
  lv_label_set_text(label, LV_SYMBOL_LEFT "  Back");
  lv_obj_set_style_text_font(label, &lv_font_montserrat_20, LV_PART_MAIN);
  lv_obj_center(label);

  ui->cam_status = lv_label_create(ui->cam);
  lv_label_set_text(ui->cam_status, "waiting for a frame");
  lv_obj_set_style_text_font(ui->cam_status, &lv_font_montserrat_20,
                             LV_PART_MAIN);
  lv_obj_set_style_text_color(ui->cam_status, lv_color_hex(0xf0f4f8),
                              LV_PART_MAIN);
  lv_obj_set_style_bg_color(ui->cam_status, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ui->cam_status, LV_OPA_60, LV_PART_MAIN);
  lv_obj_set_style_pad_all(ui->cam_status, 8, LV_PART_MAIN);
  lv_obj_set_style_radius(ui->cam_status, 8, LV_PART_MAIN);
  lv_obj_align(ui->cam_status, LV_ALIGN_BOTTOM_MID, 0, -16);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  lv_nuttx_dsc_t info;
  lv_nuttx_result_t result;

  if (lv_is_initialized())
    {
      fprintf(stderr, "ampui: LVGL is already initialised in this process\n");
      return EXIT_FAILURE;
    }

  memset(&g_ui, 0, sizeof(g_ui));
  g_ui.camfd = -1;

  lv_init();

  lv_nuttx_dsc_init(&info);
  lv_nuttx_init(&info, &result);

  if (result.disp == NULL)
    {
      fprintf(stderr, "ampui: no display - is /dev/fb0 present?\n");
      lv_deinit();
      return EXIT_FAILURE;
    }

  /* Said out loud, because "the touch device was missing" and "Linux stopped
   * forwarding touch" produce the same unresponsive screen and have different
   * causes. lv_nuttx_init() only logs this through LVGL's own log.
   */

  if (result.indev == NULL)
    {
      fprintf(stderr, "ampui: no input device - the screen will not respond to"
                      " touch\n");
    }

  printf("ampui: %" LV_PRId32 "x%" LV_PRId32 " display, camera from %s\n",
         lv_display_get_horizontal_resolution(result.disp),
         lv_display_get_vertical_resolution(result.disp),
         AMPUI_CAM_DEV);

  ampui_build_home(&g_ui);
  ampui_build_cam(&g_ui);

  /* Created paused. The camera screen is not the one being shown, and a pump
   * running against a hidden canvas is two megabytes a frame of nothing.
   */

  g_ui.frame_timer = lv_timer_create(ampui_frame_cb, AMPUI_FRAME_MS, &g_ui);
  lv_timer_pause(g_ui.frame_timer);

  g_ui.beat_timer = lv_timer_create(ampui_beat_cb, AMPUI_BEAT_MS, &g_ui);

  lv_screen_load(g_ui.home);

  for (; ; )
    {
      uint32_t idle = lv_timer_handler();

      usleep((idle ? idle : 1) * 1000);
    }

  return EXIT_SUCCESS;
}
