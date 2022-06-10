/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include <ztest.h>
#include <drivers/flash.h>

#include <storage/stream_flash.h>

#define BUF_LEN 512
#define MAX_PAGE_SIZE 0x1000 /* Max supported page size to run test on */
#define MAX_NUM_PAGES 4      /* Max number of pages used in these tests */
#define TESTBUF_SIZE (MAX_PAGE_SIZE * MAX_NUM_PAGES)
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)
#define FLASH_SIZE DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_NAME DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL

/* so that we don't overwrite the application when running on hw */
#define FLASH_BASE (64*1024)
#define FLASH_AVAILABLE (FLASH_SIZE-FLASH_BASE)

static const struct device *fdev;
static const struct flash_driver_api *api;
static const struct flash_pages_layout *layout;
static size_t layout_size;
static struct stream_flash_ctx ctx;
static int page_size;
static uint8_t *cb_buf;
static size_t cb_len;
static size_t cb_offset;
static int cb_ret;

static uint8_t buf[BUF_LEN];
static uint8_t read_buf[TESTBUF_SIZE];
const static uint8_t write_buf[TESTBUF_SIZE] = {[0 ... TESTBUF_SIZE - 1] = 0xaa};
static uint8_t written_pattern[TESTBUF_SIZE] = {[0 ... TESTBUF_SIZE - 1] = 0xaa};
static uint8_t erased_pattern[TESTBUF_SIZE]  = {[0 ... TESTBUF_SIZE - 1] = 0xff};

#define VERIFY_BUF(start, size, buf) \
do { \
	rc = flash_read(fdev, FLASH_BASE + start, read_buf, size); \
	zassert_equal(rc, 0, "should succeed"); \
	zassert_mem_equal(read_buf, buf, size, "should equal %s", #buf);\
} while (0)

#define VERIFY_WRITTEN(start, size) VERIFY_BUF(start, size, written_pattern)
#define VERIFY_ERASED(start, size) VERIFY_BUF(start, size, erased_pattern)

int stream_flash_callback(uint8_t *buf, size_t len, size_t offset)
{
	if (cb_buf) {
		zassert_equal(cb_buf, buf, "incorrect buf");
		zassert_equal(cb_len, len, "incorrect length");
		zassert_equal(cb_offset, offset, "incorrect offset");
	}

	return cb_ret;
}

static void erase_flash(void)
{
	int rc;

	rc = flash_write_protection_set(fdev, false);
	zassert_equal(rc, 0, "should succeed");

	for (int i = 0; i < MAX_NUM_PAGES; i++) {
		rc = flash_erase(fdev,
				 FLASH_BASE + (i * layout->pages_size),
				 layout->pages_size);
		zassert_equal(rc, 0, "should succeed");
	}

	rc = flash_write_protection_set(fdev, true);
	zassert_equal(rc, 0, "should succeed");
}


static void init_target(void)
{
	int rc;

	/* Ensure that target is clean */
	memset(&ctx, 0, sizeof(ctx));
	memset(buf, 0, BUF_LEN);

	/* Disable callback tests */
	cb_len = 0;
	cb_offset = 0;
	cb_buf = NULL;
	cb_ret = 0;

	erase_flash();

	rc = stream_flash_init(&ctx, fdev, buf, BUF_LEN, FLASH_BASE, 0,
			       stream_flash_callback);
	zassert_equal(rc, 0, "expected success");
}

static void test_stream_flash_init(void)
{
	int rc;

	init_target();

	/* End address out of range */
	rc = stream_flash_init(&ctx, fdev, buf, BUF_LEN, FLASH_BASE,
		      FLASH_AVAILABLE + 4, NULL);
	zassert_true(rc < 0, "should fail as size is more than available");

	rc = stream_flash_init(NULL, fdev, buf, BUF_LEN, FLASH_BASE, 0, NULL);
	zassert_true(rc < 0, "should fail as ctx is NULL");

	rc = stream_flash_init(&ctx, NULL, buf, BUF_LEN, FLASH_BASE, 0, NULL);
	zassert_true(rc < 0, "should fail as fdev is NULL");

	rc = stream_flash_init(&ctx, fdev, NULL, BUF_LEN, FLASH_BASE, 0, NULL);
	zassert_true(rc < 0, "should fail as buffer is NULL");

	/* Entering '0' as flash size uses rest of flash. */
	rc = stream_flash_init(&ctx, fdev, buf, BUF_LEN, FLASH_BASE, 0, NULL);
	zassert_equal(rc, 0, "should succeed");
	zassert_equal(FLASH_AVAILABLE, ctx.available, "Wrong size");
}

static void test_stream_flash_buffered_write(void)
{
	int rc;

	init_target();

	/* Don't fill up the buffer */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN - 1, false);
	zassert_equal(rc, 0, "expected success");

	/* Verify that no data has been written */
	VERIFY_ERASED(0, BUF_LEN);

	/* Now, write the missing byte, which should trigger a dump to flash */
	rc = stream_flash_buffered_write(&ctx, write_buf, 1, false);
	zassert_equal(rc, 0, "expected success");

	VERIFY_WRITTEN(0, BUF_LEN);
}

static void test_stream_flash_buffered_write_cross_buf_border(void)
{
	int rc;

	init_target();

	/* Test when write crosses border of the buffer */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN + 128, false);
	zassert_equal(rc, 0, "expected success");

	/* 1xBuffer should be dumped to flash */
	VERIFY_WRITTEN(0, BUF_LEN);

	/* Fill rest of the buffer */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN - 128, false);
	zassert_equal(rc, 0, "expected success");
	VERIFY_WRITTEN(BUF_LEN, BUF_LEN);

	/* Fill half of the buffer */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN/2, false);
	zassert_equal(rc, 0, "expected success");

	/* Flush the buffer */
	rc = stream_flash_buffered_write(&ctx, write_buf, 0, true);
	zassert_equal(rc, 0, "expected success");

	/* Two and a half buffers should be written */
	VERIFY_WRITTEN(0, BUF_LEN * 2 + BUF_LEN / 2);
}

static void test_stream_flash_buffered_write_unaligned(void)
{
	int rc;

	if (flash_get_write_block_size(fdev) == 1) {
		ztest_test_skip();
	}

	init_target();

	/* Test unaligned data size */
	rc = stream_flash_buffered_write(&ctx, write_buf, 1, true);
	zassert_equal(rc, 0, "expected success (%d)", rc);

	/* 1 byte should be dumped to flash */
	VERIFY_WRITTEN(0, 1);

	rc = stream_flash_init(&ctx, fdev, buf, BUF_LEN, FLASH_BASE + BUF_LEN,
			       0, stream_flash_callback);
	zassert_equal(rc, 0, "expected success");

	/* Test unaligned data size */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN - 1, true);
	zassert_equal(rc, 0, "expected success");

	/* BUF_LEN-1 bytes should be dumped to flash */
	VERIFY_WRITTEN(BUF_LEN, BUF_LEN - 1);
}

static void test_stream_flash_buffered_write_multi_page(void)
{
	int rc;
	int num_pages = MAX_NUM_PAGES - 1;

	init_target();

	/* Test when write spans multiple pages crosses border of page */
	rc = stream_flash_buffered_write(&ctx, write_buf,
					 (page_size * num_pages) + 128, false);
	zassert_equal(rc, 0, "expected success");

	/* First three pages should be written */
	VERIFY_WRITTEN(0, page_size * num_pages);

	/* Fill rest of the page */
	rc = stream_flash_buffered_write(&ctx, write_buf,
					 page_size - 128, false);
	zassert_equal(rc, 0, "expected success");

	/* First four pages should be written */
	VERIFY_WRITTEN(0, BUF_LEN * (num_pages + 1));
}

static void test_stream_flash_bytes_written(void)
{
	int rc;
	size_t offset;

	init_target();

	/* Verify that the offset is retained across failed downolads */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN + 128, false);
	zassert_equal(rc, 0, "expected success");

	/* First page should be written */
	VERIFY_WRITTEN(0, BUF_LEN);

	/* Fill rest of the page */
	offset = stream_flash_bytes_written(&ctx);
	zassert_equal(offset, BUF_LEN, "offset should match buf size");

	/* Fill up the buffer MINUS 128 to verify that write_buf_pos is kept */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN - 128, false);
	zassert_equal(rc, 0, "expected success");

	/* Second page should be written */
	VERIFY_WRITTEN(BUF_LEN, BUF_LEN);
}

static void test_stream_flash_buf_size_greater_than_page_size(void)
{
	int rc;

	/* To illustrate that other params does not trigger error */
	rc = stream_flash_init(&ctx, fdev, buf, 0x10, 0, 0, NULL);
	zassert_equal(rc, 0, "expected success");

	/* Only change buf_len param */
	rc = stream_flash_init(&ctx, fdev, buf, 0x10000, 0, 0, NULL);
	zassert_true(rc < 0, "expected failure");
}

static void test_stream_flash_buffered_write_callback(void)
{
	int rc;

	init_target();

	/* Trigger verification in callback */
	cb_buf = buf;
	cb_len = BUF_LEN;
	cb_offset = FLASH_BASE;

	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN + 128, false);
	zassert_equal(rc, 0, "expected success");

	cb_len = BUF_LEN;
	cb_offset = FLASH_BASE + BUF_LEN;

	/* Fill rest of the buffer */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN - 128, false);
	zassert_equal(rc, 0, "expected success");
	VERIFY_WRITTEN(BUF_LEN, BUF_LEN);

	/* Fill half of the buffer and flush it to flash */
	cb_len = BUF_LEN/2;
	cb_offset = FLASH_BASE + (2 * BUF_LEN);

	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN/2, true);
	zassert_equal(rc, 0, "expected success");

	/* Ensure that failing callback trickles up to caller */
	cb_ret = -EFAULT;
	cb_buf = NULL; /* Don't verify other parameters of the callback */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN, true);
	zassert_equal(rc, -EFAULT, "expected failure from callback");
}

static void test_stream_flash_flush(void)
{
	int rc;

	init_target();

	/* Perform flush with NULL data pointer and 0 lentgth */
	rc = stream_flash_buffered_write(&ctx, NULL, 0, true);
	zassert_equal(rc, 0, "expected success");
}

#ifdef CONFIG_STREAM_FLASH_ERASE
static void test_stream_flash_buffered_write_whole_page(void)
{
	int rc;

	init_target();

	/* Write all bytes of a page, verify that next page is not erased */

	/* First fill two pages with data */
	rc = stream_flash_buffered_write(&ctx, write_buf, page_size * 2, true);
	zassert_equal(rc, 0, "expected success");

	VERIFY_WRITTEN(0, page_size);
	VERIFY_WRITTEN(page_size, page_size);

	/* Reset stream_flash context */
	memset(&ctx, 0, sizeof(ctx));
	memset(buf, 0, BUF_LEN);
	rc = stream_flash_init(&ctx, fdev, buf, BUF_LEN, FLASH_BASE, 0,
			       stream_flash_callback);
	zassert_equal(rc, 0, "expected success");

	/* Write all bytes of a page, verify that next page is not erased */
	rc = stream_flash_buffered_write(&ctx, write_buf, page_size, true);
	zassert_equal(rc, 0, "expected success");

	/* Second page should not be erased */
	VERIFY_WRITTEN(page_size, page_size);
}

static void test_stream_flash_erase_page(void)
{
	int rc;

	init_target();

	/* Write out one buf */
	rc = stream_flash_buffered_write(&ctx, write_buf, BUF_LEN, false);
	zassert_equal(rc, 0, "expected success");

	rc = stream_flash_erase_page(&ctx, FLASH_BASE);
	zassert_equal(rc, 0, "expected success");

	VERIFY_ERASED(FLASH_BASE, page_size);
}
#else
static void test_stream_flash_erase_page(void)
{
	ztest_test_skip();
}

static void test_stream_flash_buffered_write_whole_page(void)
{
	ztest_test_skip();
}
#endif

void test_main(void)
{
	fdev = device_get_binding(FLASH_NAME);
	api = fdev->api;
	api->page_layout(fdev, &layout, &layout_size);

	page_size = layout->pages_size;
	__ASSERT_NO_MSG(page_size > BUF_LEN);

	ztest_test_suite(lib_stream_flash_test,
	     ztest_unit_test(test_stream_flash_init),
	     ztest_unit_test(test_stream_flash_buffered_write),
	     ztest_unit_test(test_stream_flash_buffered_write_cross_buf_border),
	     ztest_unit_test(test_stream_flash_buffered_write_unaligned),
	     ztest_unit_test(test_stream_flash_buffered_write_multi_page),
	     ztest_unit_test(test_stream_flash_buf_size_greater_than_page_size),
	     ztest_unit_test(test_stream_flash_buffered_write_callback),
	     ztest_unit_test(test_stream_flash_flush),
	     ztest_unit_test(test_stream_flash_buffered_write_whole_page),
	     ztest_unit_test(test_stream_flash_erase_page),
	     ztest_unit_test(test_stream_flash_bytes_written)
	 );

	ztest_run_test_suite(lib_stream_flash_test);
}
