#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

struct LIST {
	uint32_t list_size;
	uint32_t list_size_index;
};

struct RIFF {
	FILE *f;
	struct LIST *l;
	uint32_t file_size;
};

bool riff_open(struct RIFF **ctx_in, unsigned char *file_type, char *file_name)
{
	struct RIFF *ctx = *ctx_in = calloc(1, sizeof(struct RIFF));

	ctx->f = fopen(file_name, "wb");
	if (!ctx->f) {
		free(ctx);
		*ctx_in = NULL;
		return false;
	}

	fwrite("RIFF", 1, 4, ctx->f);
	fwrite(&ctx->file_size, 4, 1, ctx->f);
	fwrite(file_type, 1, 4, ctx->f);

	ctx->file_size = 4;

	return true;
}

bool riff_list_start(struct RIFF *ctx, unsigned char *list_type) {
	if (!ctx || !ctx->f || ctx->l)
		return false;

	ctx->l = calloc(1, sizeof(struct LIST));
	fwrite("LIST", 1, 4, ctx->f);
	ctx->l->list_size_index = ftell(ctx->f);
	fwrite(&ctx->l->list_size, 4, 1, ctx->f);
	fwrite(list_type, 1, 4, ctx->f);

	ctx->l->list_size = 4;
	ctx->file_size += 12;

	return true;
}

void riff_list_end(struct RIFF *ctx)
{
	if (!ctx || !ctx->f || !ctx->l)
		return;

	fseek(ctx->f, ctx->l->list_size_index, SEEK_SET);
	fwrite(&ctx->l->list_size, 4, 1, ctx->f);
	fseek(ctx->f, 0, SEEK_END);
	free(ctx->l);
	ctx->l = NULL;

}

bool riff_write_chunk(struct RIFF *ctx, unsigned char *chunk_type, uint32_t data_size, uint8_t *data)
{
	if (!ctx || !ctx->f)
		return false;
	
	fwrite(chunk_type, 1, 4, ctx->f);
	fwrite(&data_size, 4, 1, ctx->f);
	fwrite(data, 1, data_size, ctx->f);
	if (ctx->l)
		ctx->l->list_size += data_size + 8;

	ctx->file_size += data_size + 8;

	return true;
}

void riff_close(struct RIFF **ctx_in)
{
	struct RIFF *ctx = *ctx_in;

	if (!ctx || !ctx->f)
		return;

	if (ctx->l)
		riff_list_end(ctx);
	
	fseek(ctx->f, 4, SEEK_SET);
	fwrite(&ctx->file_size, 4, 1, ctx->f);
	fclose(ctx->f);
	free(ctx);

	*ctx_in = NULL;
}