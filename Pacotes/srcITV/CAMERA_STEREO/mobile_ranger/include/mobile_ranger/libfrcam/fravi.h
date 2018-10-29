///////////////////////////////////////////////////////////////////////////
// Copyright (c) 2005 Focus Robotics. All rights reserved. 
//
// Created by    :  Jason Peck based on GPL mmavi on www.michealmin.com
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the 
// Free Software Foundation; either version 2 of the License, or (at your 
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but 
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
// General Public License for more details.
//
///////////////////////////////////////////////////////////////////////////

#ifndef FRAVI_H
#define FRAVI_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "cxcore.h"


class fravi_container_chunk
{
	public:
	char chunk_name[4];
	int chunk_length;
	char content_type[4];
};

class fravi_avi_header
{
	public:
	char chunk_name[4];
	int chunk_length;
	int microseconds_per_frame;
	int max_bytes_per_second;
	int reserved1;
	int flags;
	int total_frames;
	int initial_frames;
	int streams;
	int suggested_buffer_size;
	int width;
	int height;
	int scale;
	int rate;
	int start;
	int length;

	fravi_avi_header() { memset(this, 0, sizeof(*this)); }
	int list()
	{
		printf("  AVI Header (%d/%d bytes)\n", chunk_length, sizeof(*this));
		printf("    ms/frame: %d\n", microseconds_per_frame);
		printf("    bytes/sec: %d\n", max_bytes_per_second);
		printf("    flags: %x\n", flags);
		printf("    total frames: %d\n", total_frames);
		printf("    initial frames: %d\n", initial_frames);
		printf("    streams: %d\n", streams);
		printf("    suggested buffer: %d\n", suggested_buffer_size);
		printf("    dimensions: %d x %d\n", width, height);
		if (scale)
			printf("    rate %d / scale %d = %d ticks/second\n", rate, scale, rate / scale);
		printf("    start: %d\n", start);
		printf("    length: %d\n", length);
		return 0;
	}
};

class fravi_stream_header
{
	public:
	char chunk_name[4];
	int chunk_length;
	char stream_type[4];
	char stream_handler[4];
	int flags;
	int reserved;
	int initial_frames;
	int scale;
	int rate;
	int start;
	int length;
	int suggested_buffer_size;
	int quality;
	int sample_size;
	int reserved2[2];

	fravi_stream_header() { memset(this, 0, sizeof(*this)); }
	int list()
	{
		printf("  Stream (%d/%d bytes)\n", chunk_length, sizeof(*this));
		printf("    Type: %.4s, handler %.4s\n", stream_type, stream_handler);
		printf("    flags: %d\n", flags);
		printf("    initial frames: %d\n", initial_frames);
		printf("    rate %d / scale %d = %d ticks/second\n", rate, scale, rate/ scale);
		printf("    start: %d\n", start);
		printf("    length: %d ticks\n", length);
		printf("    suggested buffer: %d\n", suggested_buffer_size);
		printf("    quality: %d\n", quality);
		printf("    sample size: %d\n", sample_size);
		return 0;
	}
};

class fravi_image_format // bitmapinfoheader
{
	public:
	char chunk_name[4];
	int chunk_length;
	int structure_size;
	int width;
	int height;
	short planes; // always 1?
	short depth;
	int compression; // 0=rgb, 1=8-bit indexed, 2=4-bit indexed, 3=16/32bit rgb, 4=jpeg, 5=png
	int byte_size;
	int pixels_per_meter;
	int colors_used;
	int minimum_colors;
	char color_table[]; // array of b/g/r/a bytes

	fravi_image_format() { memset(this, 0, sizeof(*this)); }
	int list()
	{
		printf("  Image format (%d/%d bytes)\n", chunk_length, chunk_length + 8);
		printf("    dimensions: %d x %d\n", width, height);
		printf("    structure size: %d\n", structure_size);
		printf("    planes: %d\n", planes);
		printf("    depth: %d\n", depth);
		printf("    compression algorithm: %d\n", compression);
		printf("    size: %d bytes\n", byte_size);
		printf("    resolution: %d/meter\n", pixels_per_meter);
		printf("    colors: %d (min %d)\n", colors_used, minimum_colors);
		return 0;
	}
};

class fravi_sound_format // waveformatex
{
	public:
	char chunk_title[4];
        unsigned int chunk_length;
        unsigned short format_type;
        unsigned short channel_numbers;
        unsigned int sample_rate;
        unsigned int bytes_per_second;
        unsigned short bytes_per_sample;
        unsigned short bits_per_sample;

	fravi_sound_format() { memset(this, 0, sizeof(*this)); }
	int list()
	{
		printf("  WAV Information (%d/%d bytes)\n", chunk_length, sizeof(*this));
		printf("    format: %d\n", format_type);
		printf("    channels: %d\n", channel_numbers);
		printf("    rate: %d\n", sample_rate);
		printf("    bytes/second: %d\n", bytes_per_second);
		printf("    bits/sample: %d\n", bits_per_sample);
		return 0;
	}
};

class fravi_file
{
	public:
	fravi_file() { memset(this, 0, sizeof(*this)); frames_per_second = 18; }
	int open_file(const char *filename, int is_black_and_white, int frames_per_second);
	int add_frame(IplImage *frame);
	int close_file();

	private:
	int descriptor;
	int frame_count;
	int frame_width;
	int frame_height;
	int is_black_and_white;
	int frames_per_second;

	int write_header();
};

class fravi_input_file
{
	public:
	fravi_input_file() { memset(this, 0, sizeof(*this)); }
	int open_file(const char *filename);
	IplImage* get_frame();
	int close_file();
	int get_frames_per_second();
	int get_width();
	int get_height();

	private:
	int			descriptor;
	int			frame_count;
	fravi_avi_header	avi_header;
	fravi_stream_header	stream_header;
	fravi_image_format	image_format;
	fravi_sound_format	sound_format; 
	IplImage		*frame;
	unsigned int		frame_length;
};

int fravi_list(char *filename);

#endif

