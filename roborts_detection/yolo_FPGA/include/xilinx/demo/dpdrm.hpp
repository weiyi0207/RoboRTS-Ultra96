/*

  Some source codes below are copied https://github.com/dvdhrm/docs/tree/master/drm-howto.

  They are modified.


  Copyright 2012-2017 David Herrmann <dh.herrmann@gmail.com>

Permission to use, copy, modify, and/or distribute this software for
any purpose with or without fee is hereby granted, provided that the
above copyright notice and this permission notice appear in all
copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
PERFORMANCE OF THIS SOFTWARE.

*/
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

namespace xilinx {
namespace demo {
struct modeset_buf;
struct modeset_dev;
static int modeset_find_crtc(int fd, drmModeRes *res, drmModeConnector *conn,
                             struct modeset_dev *dev);
static int modeset_create_fb(int fd, struct modeset_buf *buf);
static void modeset_destroy_fb(int fd, struct modeset_buf *buf);
static int modeset_setup_dev(int fd, drmModeRes *res, drmModeConnector *conn,
                             struct modeset_dev *dev);
static int modeset_open(int *out, const char *node);
static int modeset_prepare(int fd);
static void modeset_cleanup(int fd);

static int modeset_open(int *out, const char *node) {
  int fd, ret;
  uint64_t has_dumb;

  fd = open(node, O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    ret = -errno;
    fprintf(stderr, "cannot open '%s': %m\n", node);
    return ret;
  }

  if (drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &has_dumb) < 0 || !has_dumb) {
    fprintf(stderr, "drm device '%s' does not support dumb buffers\n", node);
    close(fd);
    return -EOPNOTSUPP;
  }

  *out = fd;
  return 0;
}

struct modeset_buf {
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t size;
  uint32_t handle;
  uint8_t *map;
  uint32_t fb;
};

struct modeset_dev {
  struct modeset_dev *next;

  unsigned int front_buf;
  struct modeset_buf bufs[2];

  drmModeModeInfo mode;
  uint32_t conn;
  uint32_t crtc;
  drmModeCrtc *saved_crtc;

  bool pflip_pending;
  bool cleanup;

  uint8_t r, g, b;
  bool r_up, g_up, b_up;
};

static struct modeset_dev *modeset_list = NULL;


static int modeset_prepare(int fd) {
  drmModeRes *res;
  drmModeConnector *conn;
  unsigned int i;
  struct modeset_dev *dev;
  int ret;

  /* retrieve resources */
  res = drmModeGetResources(fd);
  if (!res) {
    fprintf(stderr, "cannot retrieve DRM resources (%d): %m\n", errno);
    return -errno;
  }

  /* iterate all connectors */
  for (i = 0; i < (unsigned) res->count_connectors; ++i) {
    /* get information for each connector */
    conn = drmModeGetConnector(fd, res->connectors[i]);
    if (!conn) {
      fprintf(stderr, "cannot retrieve DRM connector %u:%u (%d): %m\n", i,
              res->connectors[i], errno);
      continue;
    }

    /* create a device structure */
    dev = (modeset_dev*) malloc(sizeof(*dev));
    memset(dev, 0, sizeof(*dev));
    dev->conn = conn->connector_id;

    /* call helper function to prepare this connector */
    ret = modeset_setup_dev(fd, res, conn, dev);
    if (ret) {
      if (ret != -ENOENT) {
        errno = -ret;
        fprintf(stderr, "cannot setup device for connector %u:%u (%d): %m\n", i,
                res->connectors[i], errno);
      }
      free(dev);
      drmModeFreeConnector(conn);
      continue;
    }

    /* free connector data and link device into global list */
    drmModeFreeConnector(conn);
    dev->next = modeset_list;
    modeset_list = dev;
  }

  /* free resources again */
  drmModeFreeResources(res);
  return 0;
}

static int modeset_setup_dev(int fd, drmModeRes *res, drmModeConnector *conn,
                             struct modeset_dev *dev) {
  int ret;

  /* check if a monitor is connected */
  if (conn->connection != DRM_MODE_CONNECTED) {
    fprintf(stderr, "ignoring unused connector %u\n", conn->connector_id);
    return -ENOENT;
  }

  /* check if there is at least one valid mode */
  if (conn->count_modes == 0) {
    fprintf(stderr, "no valid mode for connector %u\n", conn->connector_id);
    return -EFAULT;
  }

  /* copy the mode information into our device structure and into both
   * buffers */
  memcpy(&dev->mode, &conn->modes[0], sizeof(dev->mode));
  dev->bufs[0].width = conn->modes[0].hdisplay;
  dev->bufs[0].height = conn->modes[0].vdisplay;
  dev->bufs[1].width = conn->modes[0].hdisplay;
  dev->bufs[1].height = conn->modes[0].vdisplay;
  fprintf(stderr, "mode for connector %u is %ux%u\n", conn->connector_id,
          dev->bufs[0].width, dev->bufs[0].height);

  /* find a crtc for this connector */
  ret = modeset_find_crtc(fd, res, conn, dev);
  if (ret) {
    fprintf(stderr, "no valid crtc for connector %u\n", conn->connector_id);
    return ret;
  }

  /* create framebuffer #1 for this CRTC */
  ret = modeset_create_fb(fd, &dev->bufs[0]);
  if (ret) {
    fprintf(stderr, "cannot create framebuffer for connector %u\n",
            conn->connector_id);
    return ret;
  }

  /* create framebuffer #2 for this CRTC */
  ret = modeset_create_fb(fd, &dev->bufs[1]);
  if (ret) {
    fprintf(stderr, "cannot create framebuffer for connector %u\n",
            conn->connector_id);
    modeset_destroy_fb(fd, &dev->bufs[0]);
    return ret;
  }

  return 0;
}

static int modeset_find_crtc(int fd, drmModeRes *res, drmModeConnector *conn,
                             struct modeset_dev *dev) {
  drmModeEncoder *enc;
  unsigned int i, j;
  int32_t crtc;
  struct modeset_dev *iter;

  /* first try the currently conected encoder+crtc */
  if (conn->encoder_id)
    enc = drmModeGetEncoder(fd, conn->encoder_id);
  else
    enc = NULL;

  if (enc) {
    if (enc->crtc_id) {
      crtc = enc->crtc_id;
      for (iter = modeset_list; iter; iter = iter->next) {
        if (iter->crtc == (unsigned) crtc) {
          crtc = -1;
          break;
        }
      }

      if (crtc >= 0) {
        drmModeFreeEncoder(enc);
        dev->crtc = crtc;
        return 0;
      }
    }

    drmModeFreeEncoder(enc);
  }

  /* If the connector is not currently bound to an encoder or if the
   * encoder+crtc is already used by another connector (actually unlikely
   * but lets be safe), iterate all other available encoders to find a
   * matching CRTC. */
  for (i = 0; i < (unsigned) conn->count_encoders; ++i) {
    enc = drmModeGetEncoder(fd, conn->encoders[i]);
    if (!enc) {
      fprintf(stderr, "cannot retrieve encoder %u:%u (%d): %m\n", i,
              conn->encoders[i], errno);
      continue;
    }

    /* iterate all global CRTCs */
    for (j = 0; j < (unsigned) res->count_crtcs; ++j) {
      /* check whether this CRTC works with the encoder */
      if (!(enc->possible_crtcs & (1 << j))) continue;

      /* check that no other device already uses this CRTC */
      crtc = res->crtcs[j];
      for (iter = modeset_list; iter; iter = iter->next) {
        if (iter->crtc == (unsigned) crtc) {
          crtc = -1;
          break;
        }
      }

      /* we have found a CRTC, so save it and return */
      if (crtc >= 0) {
        drmModeFreeEncoder(enc);
        dev->crtc = crtc;
        return 0;
      }
    }

    drmModeFreeEncoder(enc);
  }

  fprintf(stderr, "cannot find suitable CRTC for connector %u\n",
          conn->connector_id);
  return -ENOENT;
}

static int modeset_create_fb(int fd, struct modeset_buf *buf) {
  struct drm_mode_create_dumb creq;
  struct drm_mode_destroy_dumb dreq;
  struct drm_mode_map_dumb mreq;
  int ret;

  /* create dumb buffer */
  memset(&creq, 0, sizeof(creq));
  creq.width = buf->width;
  creq.height = buf->height;
  creq.bpp = 24;
  ret = drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);
  if (ret < 0) {
    fprintf(stderr, "cannot create dumb buffer (%d): %m\n", errno);
    return -errno;
  }
  buf->stride = creq.pitch;
  buf->size = creq.size;
  buf->handle = creq.handle;

  /* create framebuffer object for the dumb-buffer */
  ret = drmModeAddFB(fd, buf->width, buf->height, 24, 24, buf->stride,
                     buf->handle, &buf->fb);
  if (ret) {
    fprintf(stderr, "cannot create framebuffer (%d): %m\n", errno);
    ret = -errno;
    goto err_destroy;
  }

  /* prepare buffer for memory mapping */
  memset(&mreq, 0, sizeof(mreq));
  mreq.handle = buf->handle;
  ret = drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);
  if (ret) {
    fprintf(stderr, "cannot map dumb buffer (%d): %m\n", errno);
    ret = -errno;
    goto err_fb;
  }

  /* perform actual memory mapping */
  buf->map = (uint8_t*)
      mmap(0, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mreq.offset);
  if (buf->map == MAP_FAILED) {
    fprintf(stderr, "cannot mmap dumb buffer (%d): %m\n", errno);
    ret = -errno;
    goto err_fb;
  }

  /* clear the framebuffer to 0 */
  memset(buf->map, 0, buf->size);

  return 0;

err_fb:
  drmModeRmFB(fd, buf->fb);
err_destroy:
  memset(&dreq, 0, sizeof(dreq));
  dreq.handle = buf->handle;
  drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
  return ret;
}

static void modeset_destroy_fb(int fd, struct modeset_buf *buf) {
  struct drm_mode_destroy_dumb dreq;

  /* unmap buffer */
  munmap(buf->map, buf->size);

  /* delete framebuffer */
  drmModeRmFB(fd, buf->fb);

  /* delete dumb buffer */
  memset(&dreq, 0, sizeof(dreq));
  dreq.handle = buf->handle;
  drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
}

inline int modeset_init() {
  int ret, fd = 0;
  const char *card;
  struct modeset_dev *iter;
  struct modeset_buf *buf;

  /* check which DRM device to open */
  card = "/dev/dri/card0";

  fprintf(stderr, "using card '%s'\n", card);

  /* open the DRM device */
  ret = modeset_open(&fd, card);
  if (ret) goto out_return;

  /* prepare all connectors and CRTCs */
  ret = modeset_prepare(fd);
  if (ret) goto out_close;

  /* perform actual modesetting on each found connector+CRTC */
  for (iter = modeset_list; iter; iter = iter->next) {
    iter->saved_crtc = drmModeGetCrtc(fd, iter->crtc);
    buf = &iter->bufs[iter->front_buf];
    ret = drmModeSetCrtc(fd, iter->crtc, buf->fb, 0, 0, &iter->conn, 1,
                         &iter->mode);
    if (ret)
      fprintf(stderr, "cannot set CRTC for connector %u (%d): %m\n", iter->conn,
              errno);
  }
  fprintf(stderr, "press RETURN to continue...\n");
  getchar();
  return fd;
out_close:
  close(fd);
out_return:
  if (ret) {
    errno = -ret;
    fprintf(stderr, "modeset failed with error %d: %m\n", errno);
  } else {
    fprintf(stderr, "exiting\n");
  }
  return ret;
}

inline int modeset_exit(int fd) {
  /* /\* draw some colors for 5seconds *\/ */
  /* modeset_draw(fd); */

  /* cleanup everything */
  modeset_cleanup(fd);
  return 0;
}

static void modeset_page_flip_event(int fd, unsigned int frame,
                                    unsigned int sec, unsigned int usec,
                                    void *data) {
  struct modeset_dev *dev = (modeset_dev *)data;

  dev->pflip_pending = false;
}

inline void modeset_update(int fd) {
  int ret;
  drmEventContext ev;
  fd_set fds;
  struct modeset_dev *dev;
  struct modeset_buf *buf;
  FD_ZERO(&fds);

  /* Set this to only the latest version you support. Version 2
   * introduced the page_flip_handler, so we use that. */
  ev.version = 2;
  ev.page_flip_handler = modeset_page_flip_event;
  dev = modeset_list;
  buf = &dev->bufs[dev->front_buf ^ 1];
  ret = drmModePageFlip(fd, dev->crtc, buf->fb, DRM_MODE_PAGE_FLIP_EVENT, dev);
  if (ret) {
    fprintf(stderr, "cannot flip CRTC for connector %u (%d): %m\n", dev->conn,
            errno);
  } else {
    dev->front_buf ^= 1;
    dev->pflip_pending = true;
  }
  FD_SET(fd, &fds);
  ret = select(fd + 1, &fds, NULL, NULL, NULL);
  if (FD_ISSET(fd, &fds)) {
    drmHandleEvent(fd, &ev);
  }
}

static void modeset_cleanup(int fd) {
  struct modeset_dev *iter;
  drmEventContext ev;
  int ret;

  /* init variables */
  memset(&ev, 0, sizeof(ev));
  ev.version = DRM_EVENT_CONTEXT_VERSION;
  ev.page_flip_handler = modeset_page_flip_event;

  while (modeset_list) {
    /* remove from global list */
    iter = modeset_list;
    modeset_list = iter->next;

    /* if a pageflip is pending, wait for it to complete */
    iter->cleanup = true;
    fprintf(stderr, "wait for pending page-flip to complete...\n");
    while (iter->pflip_pending) {
      ret = drmHandleEvent(fd, &ev);
      if (ret) break;
    }

    /* restore saved CRTC configuration */
    if (!iter->pflip_pending)
      drmModeSetCrtc(fd, iter->saved_crtc->crtc_id, iter->saved_crtc->buffer_id,
                     iter->saved_crtc->x, iter->saved_crtc->y, &iter->conn, 1,
                     &iter->saved_crtc->mode);
    drmModeFreeCrtc(iter->saved_crtc);

    /* destroy framebuffers */
    modeset_destroy_fb(fd, &iter->bufs[1]);
    modeset_destroy_fb(fd, &iter->bufs[0]);

    /* free allocated memory */
    free(iter);
  }
}

inline void* modeset_get_fb() {
  struct modeset_buf *buf;
  struct modeset_dev *dev = modeset_list;
  buf = &dev->bufs[dev->front_buf ^ 1];
  return buf->map;
}
inline int modeset_get_fb_width() {
  struct modeset_buf *buf;
  struct modeset_dev *dev = modeset_list;
  buf = &dev->bufs[dev->front_buf ^ 1];
  return buf->width;
}
inline int modeset_get_fb_height() {
  struct modeset_buf *buf;
  struct modeset_dev *dev = modeset_list;
  buf = &dev->bufs[dev->front_buf ^ 1];
  return buf->height;
}

inline int modeset_get_fb_stride() {
  struct modeset_buf *buf;
  struct modeset_dev *dev = modeset_list;
  buf = &dev->bufs[dev->front_buf ^ 1];
  return buf->stride;
}

static int global_card_fd = 0;
inline void imshow_open() {
  global_card_fd = modeset_init();
}
inline void imshow_close() {
  modeset_exit(global_card_fd);
}

inline void imshow(const cv::Mat & image) {
  //
  auto data = modeset_get_fb();
  auto fb_size = cv::Size(modeset_get_fb_width(), modeset_get_fb_height());
  auto fb = cv::Mat(fb_size, CV_8UC3, data, modeset_get_fb_stride());
  auto image_size = cv::Size(std::min(fb_size.width, image.size().width),
                             std::min(fb_size.height, image.size().height));
  auto rect = cv::Rect(cv::Point(0, 0), image_size);
  image(rect).copyTo(fb(rect));
  LOG(INFO) << "fb_size = " << fb_size;
  modeset_update(global_card_fd);
}

}}
