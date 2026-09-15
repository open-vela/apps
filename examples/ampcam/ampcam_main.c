/****************************************************************************
 * apps/examples/ampcam/ampcam_main.c
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

/* Show the frames Linux is capturing, and say what the transport is doing.
 *
 * This is the consumer end of the camera path on the RK3588 EVB7 AMP setup:
 * Linux drives the imx415 through its ISP, converts each frame to XRGB8888 and
 * publishes it in the shared carveout; this core reads it out of /dev/amcam0.
 *
 * There are two modes and the statistics one matters as much as the picture. A
 * camera preview that looks wrong tells you very little - a black screen is a
 * dead sensor, a dead pipeline, a dead notification path, a bad descriptor or a
 * blit into the wrong buffer, and they are indistinguishable by eye. So -s
 * reports frames, gaps, torn reads and rejected descriptors without touching the
 * display at all, which separates "the data is not arriving" from "the data is
 * arriving and I am drawing it wrong".
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
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/video/fb.h>

#include <arch/board/board.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ampcam_fb_s
{
  int fd;
  uint8_t *mem;
  size_t len;
  uint32_t stride;
  uint32_t width;
  uint32_t height;
  uint32_t bpp;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: now_ms
 ****************************************************************************/

static uint64_t now_ms(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000 + (uint64_t)(ts.tv_nsec / 1000000);
}

/****************************************************************************
 * Name: wait_for_stream
 *
 * Description:
 *   Poll the geometry until Linux publishes something. Separate from the read
 *   loop and noisy about it, because "nothing on screen" at this stage means the
 *   Linux side is not streaming, and that is a completely different thing to
 *   investigate than a transport fault.
 *
 ****************************************************************************/

static int wait_for_stream(int fd, struct ampcam_geom_s *geom, int timeout_s)
{
  uint64_t deadline = now_ms() + (uint64_t)timeout_s * 1000;
  bool announced = false;

  for (; ; )
    {
      if (ioctl(fd, AMPCAMIOC_GETGEOM, (unsigned long)geom) < 0)
        {
          fprintf(stderr, "ampcam: GETGEOM: %s\n", strerror(errno));
          return -1;
        }

      if (geom->width != 0 && geom->height != 0)
        {
          printf("ampcam: stream is up, %" PRIu32 "x%" PRIu32
                 ", %" PRIu32 " bpp, seq %" PRIu32 "\n",
                 geom->width, geom->height, geom->bpp, geom->seq);
          return 0;
        }

      if (now_ms() >= deadline)
        {
          fprintf(stderr,
                  "ampcam: no frames after %d s. Linux is not streaming -"
                  " start the capture side and try again.\n", timeout_s);
          return -1;
        }

      if (!announced)
        {
          printf("ampcam: waiting for Linux to start streaming...\n");
          announced = true;
        }

      usleep(200000);
    }
}

/****************************************************************************
 * Name: fb_setup
 ****************************************************************************/

static int fb_setup(struct ampcam_fb_s *fb, const char *path)
{
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;

  fb->fd = open(path, O_RDWR);
  if (fb->fd < 0)
    {
      fprintf(stderr, "ampcam: %s: %s\n", path, strerror(errno));
      return -1;
    }

  if (ioctl(fb->fd, FBIOGET_VIDEOINFO, (unsigned long)&vinfo) < 0 ||
      ioctl(fb->fd, FBIOGET_PLANEINFO, (unsigned long)&pinfo) < 0)
    {
      fprintf(stderr, "ampcam: %s: cannot read fb info: %s\n",
              path, strerror(errno));
      close(fb->fd);
      return -1;
    }

  /* Only 32-bit is handled, and refusing anything else is deliberate. The whole
   * point of converting on the Linux side was that the bytes in shared memory
   * are already what the framebuffer wants; a 16-bit panel here would mean the
   * conversion belongs on the other side of the link, not bolted on in this
   * viewer.
   */

  if (pinfo.bpp != 32)
    {
      fprintf(stderr, "ampcam: %s is %u bpp, this needs 32\n",
              path, pinfo.bpp);
      close(fb->fd);
      return -1;
    }

  fb->mem = mmap(NULL, pinfo.fblen, PROT_READ | PROT_WRITE, MAP_SHARED,
                 fb->fd, 0);
  if (fb->mem == MAP_FAILED)
    {
      fprintf(stderr, "ampcam: mmap %s: %s\n", path, strerror(errno));
      close(fb->fd);
      return -1;
    }

  fb->len    = pinfo.fblen;
  fb->stride = pinfo.stride;
  fb->width  = vinfo.xres;
  fb->height = vinfo.yres;
  fb->bpp    = pinfo.bpp;

  printf("ampcam: %s is %" PRIu32 "x%" PRIu32 ", stride %" PRIu32
         ", %zu bytes mapped\n",
         path, fb->width, fb->height, fb->stride, fb->len);
  return 0;
}

/****************************************************************************
 * Name: fb_blit
 *
 * Description:
 *   Copy the frame into the first buffer at x0/y0 and publish just those rows.
 *
 *   Only the first buffer is used, and only FBIO_UPDATE is issued. The
 *   framebuffer here is double-buffered, but its driver documents that a program
 *   which only updates is writing into the buffer already on screen and gets
 *   published immediately - so taking the pan path as well would add the whole
 *   buffer-swap and vsync-wait dance for no gain in a viewer that has nothing
 *   else to do while it waits.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: fb_hline / fb_vline
 *
 * Description:
 *   Two-pixel lines, clipped to the framebuffer.
 *
 *   Clipped rather than trusted, because the coordinates come from boxes that
 *   were produced on the other core. A box one pixel outside the frame is a
 *   plausible rounding result from the scaling Linux does; writing it without
 *   clipping is a stray store into whatever follows the framebuffer.
 *
 ****************************************************************************/

static void fb_hline(struct ampcam_fb_s *fb, int x, int y, int len,
                     uint32_t colour)
{
  int t;

  for (t = 0; t < 2; t++, y++)
    {
      uint32_t *row;
      int i;

      if (y < 0 || y >= (int)fb->height)
        {
          continue;
        }

      row = (uint32_t *)(fb->mem + (size_t)y * fb->stride);

      for (i = 0; i < len; i++)
        {
          int px = x + i;

          if (px >= 0 && px < (int)fb->width)
            {
              row[px] = colour;
            }
        }
    }
}

static void fb_vline(struct ampcam_fb_s *fb, int x, int y, int len,
                     uint32_t colour)
{
  int i;

  for (i = 0; i < len; i++)
    {
      int py = y + i;
      uint32_t *row;
      int t;

      if (py < 0 || py >= (int)fb->height)
        {
          continue;
        }

      row = (uint32_t *)(fb->mem + (size_t)py * fb->stride);

      for (t = 0; t < 2; t++)
        {
          int px = x + t;

          if (px >= 0 && px < (int)fb->width)
            {
              row[px] = colour;
            }
        }
    }
}

/****************************************************************************
 * Name: fb_draw_boxes
 *
 * Description:
 *   Outline every detection over the frame just blitted, plus a bar whose
 *   length is the confidence.
 *
 *   A bar rather than a number because there is no font here. That is not a
 *   placeholder for text: a bar is read at a glance from across a desk, which
 *   is how a preview like this actually gets looked at, and it needs none of the
 *   glyph handling that would have to be thrown away when this moves to LVGL.
 *
 *   Boxes arrive in the coordinate space of the frame, so the only transform
 *   applied here is the offset this program chose when it centred the image.
 *   That split is the contract: the other side resolves the sensor, the second
 *   ISP stream and the model's letterbox, and this side adds the one thing it
 *   knows and the other side cannot see.
 *
 ****************************************************************************/

static void fb_draw_boxes(struct ampcam_fb_s *fb,
                          const struct ampcam_detect_s *det,
                          uint32_t fw, uint32_t fh, uint32_t x0, uint32_t y0)
{
  int i;

  if (det->count <= 0 || det->width == 0 || det->height == 0)
    {
      return;
    }

  for (i = 0; i < det->count; i++)
    {
      const struct ampcam_box_s *b = &det->box[i];
      uint32_t colour;
      int bx;
      int by;
      int bw;
      int bh;
      int bar;

      /* Scaled, in case the detector's coordinate space is not the frame's.
       * They are the same today - Linux publishes boxes in the published
       * frame's space - but the descriptor carries its own width and height
       * precisely so that this does not become an assumption which breaks
       * silently the first time either resolution changes.
       */

      bx = (int)x0 + (int)((uint64_t)b->x * fw / det->width);
      by = (int)y0 + (int)((uint64_t)b->y * fh / det->height);
      bw = (int)((uint64_t)b->w * fw / det->width);
      bh = (int)((uint64_t)b->h * fh / det->height);

      if (bw <= 0 || bh <= 0)
        {
          continue;
        }

      /* Green for a person, amber for anything else. Class 0 is person in the
       * COCO ordering the model was trained on, and it is the only class this
       * is actually looking for, so it gets the colour that reads as a hit.
       */

      colour = (b->cls == 0) ? 0xff00ff00u : 0xffffaa00u;

      fb_hline(fb, bx, by, bw, colour);
      fb_hline(fb, bx, by + bh - 2, bw, colour);
      fb_vline(fb, bx, by, bh, colour);
      fb_vline(fb, bx + bw - 2, by, bh, colour);

      /* Confidence, as a bar just above the box. Clamped into the frame so a
       * detection at the top edge still shows one.
       */

      bar = (int)((uint32_t)bw * b->score / 100u);
      if (bar > 0)
        {
          int bary = (by > (int)y0 + 4) ? by - 4 : by + bh;

          fb_hline(fb, bx, bary, bar, colour);
        }
    }
}

static void fb_blit(struct ampcam_fb_s *fb, const uint8_t *frame,
                    uint32_t w, uint32_t h, uint32_t x0, uint32_t y0)
{
  uint32_t srcrow = w * 4;
  uint32_t row;

  for (row = 0; row < h; row++)
    {
      memcpy(fb->mem + (size_t)(y0 + row) * fb->stride + (size_t)x0 * 4,
             frame + (size_t)row * srcrow, srcrow);
    }
}

/****************************************************************************
 * Name: fb_publish
 *
 * Description:
 *   Hand the rows just written to the panel.
 *
 *   Separate from fb_blit because the boxes have to be drawn into the same
 *   buffer between the two. Publishing the image and then publishing the boxes
 *   would put a frame on screen without its detections and then the detections
 *   a moment later, which reads as flicker on every box.
 *
 ****************************************************************************/

static void fb_publish(struct ampcam_fb_s *fb, uint32_t w, uint32_t h,
                       uint32_t x0, uint32_t y0)
{
  struct fb_area_s area;

  area.x = (fb_coord_t)x0;
  area.y = (fb_coord_t)y0;
  area.w = (fb_coord_t)w;
  area.h = (fb_coord_t)h;

  if (ioctl(fb->fd, FBIO_UPDATE, (unsigned long)&area) < 0)
    {
      /* Reported once per occurrence rather than counted and summarised,
       * because an update that fails means nothing reaches the panel at all -
       * there is no partially working state to measure.
       */

      fprintf(stderr, "ampcam: FBIO_UPDATE: %s\n", strerror(errno));
    }
}

/****************************************************************************
 * Name: report
 ****************************************************************************/

static void report(int camfd, uint32_t frames, uint64_t elapsed_ms)
{
  struct ampcam_stats_s st;

  if (ioctl(camfd, AMPCAMIOC_GETSTATS, (unsigned long)&st) < 0)
    {
      /* Said out loud rather than returned from quietly. The first version
       * returned here without a word, which meant a failing ioctl and a
       * blocked read() produced exactly the same thing on the console -
       * nothing at all - and the counters that would have named the problem
       * were the counters being suppressed.
       */

      fprintf(stderr, "ampcam: GETSTATS: %s\n", strerror(errno));
      return;
    }

  printf("ampcam: %" PRIu32 " frames in %" PRIu64 " ms (%" PRIu32
         ".%01" PRIu32 " fps) | notify %" PRIu32 " copied %" PRIu32
         " dropped %" PRIu32 " torn %" PRIu32 " bad %" PRIu32 "\n",
         frames, elapsed_ms,
         elapsed_ms ? (uint32_t)(frames * 1000 / elapsed_ms) : 0,
         elapsed_ms ? (uint32_t)((frames * 10000 / elapsed_ms) % 10) : 0,
         st.notify, st.copied, st.dropped, st.torn, st.bad);

  /* On its own line, because the two transports run at different rates and
   * folding them together would make a detector at ten a second look like a
   * frame path missing two thirds of its work.
   *
   * Printed unconditionally, including when everything is zero. The first
   * version of this collected these four counters and then never displayed
   * them, so a working transport and an absent one looked identical on the
   * console - the same failure as the report that said nothing when its ioctl
   * failed, one level up.
   */

  printf("ampcam: detect | notify %" PRIu32 " read %" PRIu32
         " torn %" PRIu32 " bad %" PRIu32 "\n",
         st.det_notify, st.det_read, st.det_torn, st.det_bad);
}

/****************************************************************************
 * Name: usage
 ****************************************************************************/

static void usage(const char *prog)
{
  fprintf(stderr,
          "Usage: %s [-s] [-n frames] [-i seconds] [-c dev] [-f dev]\n"
          "  -s          statistics only, do not touch the framebuffer\n"
          "  -n frames   stop after this many frames (0 = forever)\n"
          "  -i seconds  reporting interval, default 2\n"
          "  -c dev      camera device, default /dev/amcam0\n"
          "  -f dev      framebuffer, default /dev/fb0\n",
          prog);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  const char *camdev = "/dev/amcam0";
  const char *fbdev = "/dev/fb0";
  struct ampcam_geom_s geom;
  struct ampcam_detect_s det;
  struct ampcam_fb_s fb;
  bool det_present = false;
  bool det_complained = false;
  uint8_t *frame = NULL;
  size_t framesize;
  uint32_t limit = 0;
  uint32_t interval = 2;
  uint32_t frames = 0;
  uint32_t x0 = 0;
  uint32_t y0 = 0;
  uint64_t start;
  uint64_t last;
  bool stats_only = false;
  int camfd = -1;
  int ret = EXIT_FAILURE;
  int opt;

  memset(&fb, 0, sizeof(fb));
  memset(&det, 0, sizeof(det));
  fb.fd = -1;
  det.count = -1;

  while ((opt = getopt(argc, argv, "sn:i:c:f:h")) != -1)
    {
      switch (opt)
        {
          case 's':
            stats_only = true;
            break;

          case 'n':
            limit = (uint32_t)strtoul(optarg, NULL, 0);
            break;

          case 'i':
            interval = (uint32_t)strtoul(optarg, NULL, 0);
            if (interval == 0)
              {
                interval = 1;
              }
            break;

          case 'c':
            camdev = optarg;
            break;

          case 'f':
            fbdev = optarg;
            break;

          default:
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

  camfd = open(camdev, O_RDONLY);
  if (camfd < 0)
    {
      fprintf(stderr, "ampcam: %s: %s\n", camdev, strerror(errno));
      return EXIT_FAILURE;
    }

  if (wait_for_stream(camfd, &geom, 30) < 0)
    {
      goto out;
    }

  framesize = (size_t)geom.width * geom.height * geom.bpp;
  frame = malloc(framesize);
  if (frame == NULL)
    {
      fprintf(stderr, "ampcam: cannot allocate %zu bytes\n", framesize);
      goto out;
    }

  if (!stats_only)
    {
      if (fb_setup(&fb, fbdev) < 0)
        {
          goto out;
        }

      if (geom.width > fb.width || geom.height > fb.height)
        {
          fprintf(stderr,
                  "ampcam: frame %" PRIu32 "x%" PRIu32 " does not fit the"
                  " %" PRIu32 "x%" PRIu32 " panel - reduce the capture size on"
                  " the Linux side\n",
                  geom.width, geom.height, fb.width, fb.height);
          goto out;
        }

      x0 = (fb.width - geom.width) / 2;
      y0 = (fb.height - geom.height) / 2;

      /* Clear once so the border around the image is black rather than whatever
       * the previous occupant of the buffer left there. Only the first buffer,
       * which is the one being drawn into.
       */

      memset(fb.mem, 0, (size_t)fb.stride * fb.height);

      printf("ampcam: drawing at %" PRIu32 ",%" PRIu32 "\n", x0, y0);
    }

  /* A baseline before the first read, and this is the single most useful line
   * this program prints.
   *
   * read() blocks until there is a frame, so if the transport is failing in a
   * way that never produces one, everything after this point is silent and the
   * counters that say why are unreachable. Printing them once up front means
   * "notify climbing, copied zero, torn climbing" is on the console before
   * anything can block - which is the difference between reading the answer and
   * deducing it.
   */

  printf("ampcam: transport counters before the first read:\n");
  report(camfd, 0, 0);

  start = now_ms();
  last = start;

  for (; ; )
    {
      ssize_t n = read(camfd, (char *)frame, framesize);

      if (n < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          if (errno == ETIMEDOUT)
            {
              /* No clean frame for a couple of seconds. Not fatal - the
               * producer may simply be idle - but the counters say which it is,
               * so print them and carry on rather than either exiting or
               * sitting here quietly.
               */

              printf("ampcam: no frame for 2s -");
              report(camfd, frames, now_ms() - start);
              printf("ampcam: if torn is climbing, this side cannot copy a"
                     " frame faster than Linux replaces it\n");
              continue;
            }

          fprintf(stderr, "ampcam: read: %s\n", strerror(errno));
          goto out;
        }

      if ((size_t)n != framesize)
        {
          /* A short read means the geometry moved under us, which happens if
           * the Linux side restarts capture at a different size. Stopping with
           * the numbers is more useful than silently drawing a stretched image.
           */

          fprintf(stderr,
                  "ampcam: read %zd bytes, expected %zu - capture geometry"
                  " changed, restart this program\n", n, framesize);
          goto out;
        }

      frames++;

      /* Detections fetched every frame rather than only when a notification
       * arrived, and the reason is the rate difference. The producer publishes
       * about ten sets a second while this draws thirty frames; if the boxes
       * were only drawn on the frames where a set happened to be new, two
       * frames in three would show none and the result would flicker rather
       * than lag. Asking every frame and redrawing whatever is current makes
       * the boxes hold still between updates, which is what "the boxes are
       * slightly behind" is supposed to look like.
       */

      if (ioctl(camfd, AMPCAMIOC_GETDET, (unsigned long)&det) < 0)
        {
          if (!det_complained)
            {
              fprintf(stderr, "ampcam: GETDET: %s - boxes will not be"
                      " drawn\n", strerror(errno));
              det_complained = true;
            }

          det.count = -1;
        }

      if (!stats_only)
        {
          fb_blit(&fb, frame, geom.width, geom.height, x0, y0);

          if (det.count > 0)
            {
              fb_draw_boxes(&fb, &det, geom.width, geom.height, x0, y0);
            }

          fb_publish(&fb, geom.width, geom.height, x0, y0);
        }

      /* Said once when a detector appears and once when it goes away, because
       * the interesting transitions are exactly those two and printing per
       * frame would bury them.
       */

      if ((det.count >= 0) != det_present)
        {
          det_present = (det.count >= 0);
          printf("ampcam: detector %s\n",
                 det_present ? "present" : "gone");
        }

      if (now_ms() - last >= (uint64_t)interval * 1000)
        {
          report(camfd, frames, now_ms() - start);
          last = now_ms();
        }

      if (limit != 0 && frames >= limit)
        {
          break;
        }
    }

  report(camfd, frames, now_ms() - start);
  ret = EXIT_SUCCESS;

out:
  if (fb.mem != NULL && fb.mem != MAP_FAILED)
    {
      munmap(fb.mem, fb.len);
    }

  if (fb.fd >= 0)
    {
      close(fb.fd);
    }

  free(frame);

  if (camfd >= 0)
    {
      close(camfd);
    }

  return ret;
}
