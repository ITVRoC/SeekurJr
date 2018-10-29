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

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "fravi.h"

int fravi_file::open_file(const char *filename, int is_black_and_white, int frames_per_second)
{
	if (!this)
		return -EINVAL;

	if (frames_per_second <= 0)
	{
		fprintf(stderr, "Invalid frames per second value %d\n", frames_per_second);
		return -EINVAL;
	}

	memset(this, 0, sizeof(*this));

	this->is_black_and_white = is_black_and_white;
	this->frames_per_second = frames_per_second;

	if ((descriptor = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0664)) < 0)
	{
		int status = -errno;
		perror(filename);
		return status;
	}

	write_header();

	return descriptor;	
}


int fravi_file::add_frame(IplImage *frame)
{
  if (!this || (descriptor < 0))
    return -EINVAL;

  if (!frame_width || !frame_height)
  {
    frame_width = frame->width;
    frame_height = frame->height;
  }

  else if ((frame_width != frame->width) || (frame_height != frame->height))
  {
    fprintf(stderr, "ERROR: Frame width/height not allowed to change mid stream!\n");
    return -EINVAL;
  }

  // As with everything Micro$oft, the order of data in an RGB frame is 
  // reversed: bytes are BGR and scan from lower left to top right. 
  // Also, frame widths must be exact multiples of four.

  int length = 0;
  unsigned char *output = NULL;
  int actual_width = frame_width & 0xfffffffc;

  if (is_black_and_white)
  {
	  length = actual_width * frame_height;

	  if (frame->origin == 1) {
		  output = (unsigned char*)frame->imageData;
	  } else {
		  output = new unsigned char[length];
		  

		  for (int y = 0; y < frame_height; y++)
		  {
			  unsigned char *source = (unsigned char*)(frame->imageData + y*frame->widthStep);

			  unsigned char *destination = &output[y * actual_width];

			  for (int x = 0; x < actual_width; ++x, ++source, ++destination) {
				  if (*source == 255)
					  *source = 254; // FIXME

				  *destination = *source;
			  }
		  }

		  /* FIXME: can we avoid this copy and just assign the image to the destination?
		  for (int y = frame_height - 1; y >= 0; --y)
		  {
			  unsigned char *source = (unsigned char*)(frame->imageData + y*frame->widthStep);

			  unsigned char *destination = &output[(frame_height - y - 1) * actual_width];
			  for (int x = 0; x < actual_width; ++x, ++source, ++destination) {
				  if (*source == 255)
					  *source = 254; // FIXME

				  *destination = *source;
			  }
		  }
		  */
	  }
  }

  else // RGB
  {
	  length = actual_width * frame_height * 3;
	  if (frame->origin == 1) {
		  output = (unsigned char*)frame->imageData;
	  } else {
		  output = new unsigned char[length];
		  for (int y = 0; y < frame_height; ++y)
		  {
			  unsigned char *source = (unsigned char*)(frame->imageData + y*frame->widthStep);

			  unsigned char *destination = &output[(frame_height - y - 1) * actual_width * 3];
			  for (int x = actual_width; x; --x, source += 3, destination += 3)
			  {
				  *destination = source[2];
				  destination[1] = source[1];
				  destination[2] = *source;
			  }
		  }
	  }
  }

  char chunk_header[8];
  memcpy(chunk_header, "00db", 4);
  *((int*) &chunk_header[4]) = length;
  if ((write(descriptor, chunk_header, 8) < 0) || (write(descriptor, output, length) < 0))
  {
    int status = -errno;
    perror("Failure writing frame data");
    return status;
  }
  
  if (frame->origin == 0)
	  delete output;

  ++frame_count;

  return length;

}

int fravi_file::close_file()
{
	if (!this || (descriptor < 0))
		return -EINVAL;

	write_header();

	close(descriptor);

	descriptor = -1;

	return -EINVAL;
}

int fravi_file::write_header()
{
	if (!this || (descriptor < 0))
		return -EINVAL;

	struct stat file_info;	
	if (fstat(descriptor, &file_info) < 0)
	{
		perror("Failure finding file length for header write");
		return -1;
	}

	int header_size = 0;
	int frame_size = frame_width * frame_height * 3;
	if (is_black_and_white)
		frame_size = frame_width * frame_height;
		
	fravi_container_chunk riff_header;
	memcpy(riff_header.chunk_name, "RIFF", 4);
	riff_header.chunk_length = file_info.st_size - 8;
	memcpy(riff_header.content_type, "AVI ", 4);
	header_size += 12;

	fravi_container_chunk header_list;
	memcpy(header_list.chunk_name, "LIST", 4);
	memcpy(header_list.content_type, "hdrl", 4);
	int header_list_start = header_size + 8;
	header_size += 12;
	
		fravi_avi_header avi_header;
		memcpy(avi_header.chunk_name, "avih", 4);
		avi_header.chunk_length = sizeof(avi_header) - 8;
		avi_header.microseconds_per_frame = 1000000 / frames_per_second;
		avi_header.max_bytes_per_second = frame_size * frames_per_second;
		//avi_header.flags = 0x10; // ???
		avi_header.total_frames = frame_count;
		avi_header.streams = 1;
		avi_header.suggested_buffer_size = avi_header.max_bytes_per_second;
		avi_header.width = frame_width;
		avi_header.height = frame_height;
		avi_header.scale = 1;
		avi_header.rate = frames_per_second;
		avi_header.length = frame_count;
		header_size += sizeof(avi_header);
	
		fravi_container_chunk stream_list;
		memcpy(stream_list.chunk_name, "LIST", 4);
		memcpy(stream_list.content_type, "strl", 4);
		int stream_list_start = header_size + 8;
		header_size += 12;
	
			fravi_stream_header stream_header;
			memcpy(stream_header.chunk_name, "strh", 4);
			stream_header.chunk_length = sizeof(stream_header) - 8;
			memcpy(stream_header.stream_type, "vids", 4);
			memcpy(stream_header.stream_handler, "RLE ", 4);
			stream_header.scale = avi_header.scale;
			stream_header.rate = avi_header.rate;
			stream_header.length = avi_header.length;
			stream_header.suggested_buffer_size = avi_header.suggested_buffer_size;
			//stream_header.quality = 8500; // ??
			header_size += sizeof(stream_header);

			fravi_image_format image_format;
			memcpy(image_format.chunk_name, "strf", 4);
			image_format.chunk_length = sizeof(fravi_image_format) - 8;
			image_format.structure_size = 40;
			image_format.width = frame_width;
			image_format.height = frame_height;
			image_format.planes = 1;
			image_format.depth = 24;
			image_format.compression = 0;
			image_format.byte_size = frame_size;
			//image_format.minimum_colors = 0x1000000;
			image_format.minimum_colors = 256;
			header_size += sizeof(image_format);

			unsigned char color_table[1024];
			if (is_black_and_white)
			{
				image_format.chunk_length += 1024;
				//image_format.compression = 1; // indexed color
				image_format.depth = 8;
				image_format.minimum_colors = 256;
				header_size += 1024;
	
				for (int init = 0; init < 1024; init += 4)
				{
					color_table[init] = init / 4;
					color_table[init + 1] = init / 4;
					color_table[init + 2] = init / 4;
					color_table[init + 3] = 0;
				}
			}

		stream_list.chunk_length = header_size - stream_list_start;

	header_list.chunk_length = header_size - header_list_start;

	fravi_container_chunk movie_data_list;
	movie_data_list.chunk_length = file_info.st_size - header_size - 8;
	memcpy(movie_data_list.chunk_name, "LIST", 4);
	memcpy(movie_data_list.content_type, "movi", 4);

	if (lseek(descriptor, 0, SEEK_SET) < 0)
	{
		perror("Failure seeking file start for header write");
		return -1;
	}

	write(descriptor, &riff_header, sizeof(riff_header));
	write(descriptor, &header_list, sizeof(header_list));
	write(descriptor, &avi_header, sizeof(avi_header));
	write(descriptor, &stream_list, sizeof(stream_list));
	write(descriptor, &stream_header, sizeof(stream_header));
	write(descriptor, &image_format, sizeof(image_format));
	if (is_black_and_white)
		write(descriptor, color_table, sizeof(color_table));
	write(descriptor, &movie_data_list, sizeof(movie_data_list));

	return file_info.st_size;
}

static unsigned long str2ulong(uint8_t *str)
{
   return ( str[0] | (str[1]<<8) | (str[2]<<16) | (str[3]<<24) );
}

#define PAD_EVEN(x) ( ((x)+1) & ~1 )

IplImage* fravi_input_file::get_frame() {

  if (!this || (descriptor < 0))
	  return NULL;


  uint8_t chunk_header[8];
  if(read(descriptor, chunk_header, 8) != 8) {
	  perror("Failure reading frame data\n");
	  return NULL;
  }

  if (strncmp((char*)chunk_header, "00db", 4) != 0) {
	  perror("Failure checking chunk_header\n");
	  return NULL;
  }
  
  if (str2ulong(chunk_header+4) != frame_length) {
	  perror("Failure checking chunk_header length\n");
	  return NULL;
  }
 
  if (read(descriptor, frame->imageData, frame_length) != (signed)frame_length) {
	  perror("Error reading image\n");
	  fprintf(stderr, "frame_length=%d\n", frame_length);	 
	  return NULL;
  }
  
  ++frame_count;

  return frame;     
}

int fravi_input_file::get_frames_per_second() 
{
	return avi_header.rate;
}

int fravi_input_file::get_width() 
{
        return image_format.width;
}

int fravi_input_file::get_height() 
{
	return image_format.height;
}

int fravi_input_file::open_file(const char *filename)
{
        char empty = 0;
	char *current_stream = &empty;
	uint8_t data[256];
	
	if ((descriptor = open(filename, O_RDONLY)) < 0) {
		perror(filename);
		return -EINVAL;
	}

	while (1)
	{	       	
		int rval = read(descriptor, data, 8);
		if (rval != 8) {			
			printf("EOF %d\n", errno);
			//printf("errno=%d, %s\n", errno, strerror(errno));
			//printf("data=%s,rval=%d,errno=%d:%s\n", 
			//       (char*)data, rval, errno, strerror(errno));
			break; 
		}

		long chunk_size = str2ulong(data+4);
		chunk_size = PAD_EVEN(chunk_size);
		int length = (9 + chunk_size) & 0xfffffffe;


		if (strncmp((char*)data, "RIFF", 4) == 0)
		{
			read(descriptor, data+8, 4);
			printf("%.4s:%.4s %ld/%d bytes\n", (char*)data, (char*)data+8, chunk_size, length);			
		}
		else if (strncmp((char*) data, "LIST", 4) == 0)
		{
			read(descriptor, data+8, 4);
			printf("%.4s/%.4s: %ld/%ld bytes\n", (char*) data, (char*)data+8, 
			       chunk_size + 4, chunk_size + 12);
		}
		else if (strncmp((char*) data, "avih", 4) == 0)
		{
			lseek(descriptor, -8, SEEK_CUR);
			read(descriptor, &avi_header, sizeof(fravi_avi_header));
			lseek(descriptor, length - sizeof(fravi_avi_header), SEEK_CUR);
			avi_header.list();
		}
		else if (strncmp((char*) data, "strh", 4) == 0)
		{
			lseek(descriptor, -8, SEEK_CUR);
			read(descriptor, &stream_header, sizeof(fravi_stream_header));
			lseek(descriptor, length - sizeof(fravi_stream_header), SEEK_CUR);
			stream_header.list();
			current_stream = stream_header.stream_type;
			printf("current_stream=%s\n", current_stream);
		}
		else if ((strncmp((char*)data, "strf", 4) == 0) && (strncmp(current_stream, "vids", 4) == 0))
		{
			lseek(descriptor, -8, SEEK_CUR);
			read(descriptor, &image_format, sizeof(fravi_image_format));
			lseek(descriptor, length - sizeof(fravi_image_format), SEEK_CUR);
			image_format.list();
		}
		else if ((strncmp((char*)data, "strf", 4) == 0) && (strncmp(current_stream, "auds", 4) == 0))
		{
			lseek(descriptor, -8, SEEK_CUR);
			read(descriptor, &sound_format, sizeof(fravi_sound_format));
			lseek(descriptor, length - sizeof(fravi_sound_format), SEEK_CUR);
			sound_format.list();
		}
		else if (strncmp((char*) data, "strn", 4) == 0)
		{			
			read(descriptor, data+8, length-8);
			printf("Comment (%ld/%d bytes): %s\n", chunk_size, length, &data[8]);
		}
		else
		{					       
			printf("%.4s: %d bytes\n", (char*) data, length-8);
			/* reset to begining of image data */
			lseek(descriptor, -8, SEEK_CUR);
			break;
		}
	}

	int actual_width = image_format.width & 0xfffffffc;
	frame = NULL;
	
	if (image_format.colors_used == 0)
	{
		frame_length = actual_width * image_format.height;
		frame = cvCreateImageHeader( 
			cvSize(image_format.width, image_format.height),
			IPL_DEPTH_8U, 1 );
		frame->widthStep = actual_width;
		frame->imageSize = frame->widthStep * frame->height;
		frame->imageData = frame->imageDataOrigin = 
			(char*)cvAlloc( (size_t)frame->imageSize );		
		frame->origin = 1;
	}
	else // RGB
	{
		frame_length = actual_width * image_format.height * 3;
		perror("Not implemented yet\n");
	}	  
	
	return 0;
}

int fravi_input_file::close_file() {
	if (descriptor >= 0)
		close(descriptor);
	return 0;
}


int fravi_list2(char *filename)
{
	int descriptor = -1;
	char *buffer = NULL;
	char *scan = NULL;
	char empty = 0;
	char *current_stream = &empty;
	struct stat file_info;

	printf("IN!!\n");
	
	if ((descriptor = open(filename, O_RDONLY)) < 0)
		perror(filename);
			
	else if (fstat(descriptor, &file_info) < 0)
		perror(filename);
			
	else if ((scan = (buffer = new char[file_info.st_size])) == NULL)
		perror("Buffer allocation failure");

	else if (read(descriptor, buffer, file_info.st_size) < 0)
		perror(filename);
		
	else while (scan < &buffer[file_info.st_size])
	{
		int chunk_size = *((int*) &scan[4]);
		int length = (9 + chunk_size) & 0xfffffffe;

		if (strncmp(scan, "RIFF", 4) == 0)
		{
			printf("%.4s:%.4s %d/%d bytes\n", scan, &scan[8], chunk_size, length);
			scan += 12;
		}
		else if (strncmp((char*) scan, "LIST", 4) == 0)
		{
			printf("%.4s/%.4s: %d/%d bytes\n", (char*) scan, (char*) &scan[8], 
				chunk_size + 4, chunk_size + 12);
			scan += 12;
		}
// Error ID - 0x80040265

		else if (strncmp((char*) scan, "avih", 4) == 0)
		{
			((fravi_avi_header*) scan)->list();
			scan += length;
		}
		else if (strncmp((char*) scan, "strh", 4) == 0)
		{
			((fravi_stream_header*) scan)->list();
			current_stream = &scan[8];
			scan += length;
		}
		else if ((strncmp(scan, "strf", 4) == 0) && (strncmp(current_stream, "vids", 4) == 0))
		{
			((fravi_image_format*) scan)->list();
			scan += length;
		}

		else if ((strncmp(scan, "strf", 4) == 0) && (strncmp(current_stream, "auds", 4) == 0))
		{
			((fravi_sound_format*) scan)->list();
			scan += length;
		}
		else if (strncmp((char*) scan, "strn", 4) == 0)
		{
			printf("Comment (%d/%d bytes): %s\n", chunk_size, length, &scan[8]);
			scan += length;
		}
		else
		{
			printf("%.4s: %d bytes\n", (char*) scan, length);
			scan += length;
			break;
		}
	}

	if (descriptor >= 0)
		close(descriptor);


	printf("OUT!!\n");
	return 0;
}
