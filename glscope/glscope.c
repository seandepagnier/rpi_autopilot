/* Copyright (C) 2007 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * simple glut Opengl scope, reading data as whitespace separated ascii numbers 
 * see ./glscope -h for options
 */

#include <stdio.h>
#include <stdlib.h>

#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <stdarg.h>

#include <sys/time.h>

#include <math.h>
#include <complex.h>
#include <fftw3.h>

#include <GL/glut.h>

#define MAX_TRACES 12

#define NUM_X_DIV 5
#define NUM_Y_DIV 6

static int freeze, drawpoints;

struct datapoint {
   double time, data;
   struct datapoint *next;
};

struct trace {
   struct datapoint *datapoints;
   double offset;
   int visible;
} traces[MAX_TRACES];

double scale = 1;
int scalestate = 0;
const double colors[MAX_TRACES][3] = {{1, 0, 0}, {0, 1, 0}, {1, 1, 0},
                                      {1, 0, 1}, {0, 1, 1}, {0, 0, 1},
                                      {1, 1, 1}, {1, .5, 0}, {.5, 1, 0},
                                      {.5, .5, .5}, {0, .5, .5}, {.5, 0, 1}};

char prefix[128] = "";

static int fps = 20;

static struct datapoint *freedatapoints;
static int numtraces;

static double delay;
static double disptime = 10;

static int curtrace = 0;

static int ffton;

int fd;

static double starttime;
static double gettime(void)
{
   struct timeval tv;
   gettimeofday(&tv, NULL);
   return tv.tv_sec + (double)tv.tv_usec/1000000;
}

static double curtime(void)
{
   return gettime() - starttime;
}

static struct datapoint *allocdatapoint(void)
{
   if(freedatapoints) {
      struct datapoint *datapoint = freedatapoints;
      freedatapoints = freedatapoints->next;
      return datapoint;
   } else
      return malloc(sizeof(struct datapoint));
}

static void adddatapoint(int trace, double time, double data)
{
   struct datapoint *datapoint = allocdatapoint();
   int pos = 0;
   datapoint->time = time;
   datapoint->data = data;
   datapoint->next = traces[trace].datapoints;
   traces[trace].datapoints = datapoint;
}

static void freedatapoint(struct datapoint *datapoint)
{
   datapoint->next = freedatapoints;
   freedatapoints = datapoint;
}

static void parseline(char *line)
{
    /* hack for datagen outputs */
    if(*line == '\r')
        line++;

    if(strncmp(line, prefix, strlen(prefix)))
        return;
    line += strlen(prefix);
    
    while(*line == ' ')
       line++;
    
   int i = 0;
   char *endptr = 0;

   double time = curtime();
   double data[MAX_TRACES];

   for(;;) {
      double val = strtod(line, &endptr);
      if(endptr == line)
         break;

      if(i == numtraces) {
        numtraces++;
        traces[i].visible = 1;
      }
      data[i] = val;
      i++;
      line = endptr;
   }

   if(i != numtraces) {
      fprintf(stderr, "underflow, discarding\n");
      return;
   }
   
   for(i = 0; i<numtraces; i++)
      adddatapoint(i, time, data[i]);
}

void draw_fft(struct trace *t)
{
   struct datapoint *dp;
   /* find out how many points there are */
   int c = 0;
   for(dp = t->datapoints; dp; dp = dp->next)
      c++;

   if(c < 1)
      return;

   /* copy data into temp buffer */

   double in[c];
   c = 0;
   for(dp = t->datapoints; dp; dp = dp->next) {
      //      in[c] = dp->time;
      in[c] = dp->data - t->offset;
      c++;
   }

   complex out[c];

   fftw_plan p = fftw_plan_dft_r2c_1d(c, in, out, FFTW_ESTIMATE);

   fftw_execute(p);

   fftw_destroy_plan(p);

   glBegin(GL_LINE_STRIP);
   double norm = 0;
   int i;
   for(i = 0; i<c/2; i++) {
      double r = creal(out[i]), j = cimag(out[i]);
      norm += r*r + j*j;
   }

   norm = sqrt(norm);

   for(i = 0; i<c/2; i++) {
      double r = creal(out[i]), j = cimag(out[i]);
      glVertex2d((double)i * 2 / (c-2), sqrt(r*r + j*j) / norm);
   }
   glEnd();

}

void idle(void)
{
   if(delay) {
      static double lastread;
      double time = curtime();
      if(time - lastread < delay)
         return;
      lastread = time;
   }

   static char buf[256];
   static int off;
   while(read(fd, buf+off, 1) == 1) {
      if(buf[off] == '\n') {
         buf[off+1] = '\0';
         parseline(buf);
         off = 0;

         if(delay)
            return;
      } else
         if(++off == sizeof buf - 1) {
            fprintf(stderr, "glscope: out of buffer\n");
            off = 0;
         }
   }
}

void timeout(int arg)
{
   glutPostRedisplay();
   glutTimerFunc(arg, timeout, arg);
}

double
calctracenoise(int trace)
{
   struct datapoint *dp;
   double total = 0, count = 0;
   for(dp = traces[trace].datapoints; dp; dp = dp->next) {
      total += dp->data;
      count++;
   }

   double mean = total / count;
   total = 0;
   for(dp = traces[trace].datapoints; dp; dp = dp->next) {
      double dev = dp->data - mean;
      total += dev*dev;
   }
   return sqrt(total / count);
}

#define FONT GLUT_BITMAP_9_BY_15
void drawputs(const char *str)
{
   while(*str)
      glutBitmapCharacter(FONT, *str++);      
}

void drawprintf(const char *fmt, ...)
{
   va_list ap;
   va_start(ap, fmt);
   char buf[256];
   vsnprintf(buf, sizeof buf, fmt, ap);
   drawputs(buf);
   va_end(ap);
}

void synccolor(void)
{
   double pos[4], vp[4];
   glGetDoublev(GL_VIEWPORT, vp);
   glGetDoublev(GL_CURRENT_RASTER_POSITION, pos);
   glRasterPos2d(pos[0]/vp[2], pos[1]/vp[3]);
}

void drawticks(void)
{
   glLineWidth(1);
   glEnable(GL_LINE_STIPPLE);
   glColor3d(.6, .6, .6);
   glLineStipple(1, 0x0011);
   glBegin(GL_LINES);
   int i;
   for(i = 1; i < NUM_X_DIV; i++) {
      double x = (double)i / NUM_X_DIV;
      glVertex2d(x, 0);
      glVertex2d(x, 1);
   }

   for(i = 1; i < NUM_Y_DIV; i++) {
      double y = (double)i / NUM_Y_DIV;
      glVertex2d(0, y);
      glVertex2d(1, y);
   }

   glEnd();
   glDisable(GL_LINE_STIPPLE);
}

void drawtext(void)
{
   // For each datapoint display its scale
   glColor3d(1, 1, 1);
   glRasterPos2d(0, 0);
   int i;
   for(i = 0; i<numtraces; i++) {
      glColor3dv(colors[i]);
      synccolor();
      drawprintf("%d ", i+1);
   }

   glColor3dv(colors[curtrace]);
   synccolor();

   struct trace *t = &traces[curtrace];
   double val = 0.0/0.0;
   if(t->datapoints)
      val = t->datapoints->data;
   drawprintf("offset: %g  value: %g  visible: %s  noise: %g  ", t->offset,
              val, t->visible ? "true" : "false", calctracenoise(curtrace));
   
   glColor3d(1, 1, 1);
   synccolor();
   drawprintf("scale: %g  time: %g", scale, disptime);

   if(ffton)
      draw_fft(t);
}

void tracevertexes(int trace, double time)
{
   /* remove datapoints after the first one that is off the screen */
   struct datapoint **datapoint = &traces[trace].datapoints;
   int first = 1;
   while(*datapoint)
      if((*datapoint)->time < time - disptime && (first ? (first = 0) : 1)) {
         struct datapoint *freed = *datapoint;
         *datapoint = (*datapoint)->next;
         freedatapoint(freed);
      } else {
         glVertex2d((*datapoint)->time-time, (*datapoint)->data);
         datapoint = &(*datapoint)->next;
      }
}

void drawtrace(int trace, double time)
{
   glPushMatrix();

   glTranslated(0, .5, 0); /* center on 0 */
   glScaled(1, 2/(scale * NUM_Y_DIV), 1);


   glTranslated(0, -traces[trace].offset, 0);

   glColor3dv(colors[trace]);
   glBegin(GL_LINE_STRIP);
   tracevertexes(trace, time);
   glEnd();
   if(drawpoints) {
      glPointSize(8);
      glBegin(GL_POINTS);
      tracevertexes(trace, time);
      glEnd();
   }
   glPopMatrix();
}

void display(void)
{
   if(freeze)
      return;

   glClear (GL_COLOR_BUFFER_BIT);

   /* update current time */
   double time = curtime();

   drawticks();
   drawtext();

   glPushMatrix();

   glScaled(1/disptime, .5, 1);
   glTranslated(disptime, .5, 0);

   glLineWidth(3);

   int i;
   for(i=0; i<numtraces; i++)
      if(traces[i].visible)
         drawtrace(i, time);

   glPopMatrix();

   glutSwapBuffers();
}

void reshape (int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   glMatrixMode (GL_MODELVIEW);
   glLoadIdentity();
   gluOrtho2D(0, 1, 0, 1);
}

void initgl(void)
{
   glClearColor (0.0, 0.0, 0.0, 0.0);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

   glEnable(GL_LINE_SMOOTH);
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

static void
inittraces(void)
{
   int i;
   for(i = 0; i<MAX_TRACES; i++)
      traces[i].offset = 0;
}

static void increasescale(void)
{
   if(scalestate%3 == 1)
      scale *= 2.5;
   else
      scale *= 2;
   scalestate++;
}

static void decreasescale(void)
{
   if(scalestate%3 == 2)
      scale /= 2.5;
   else
      scale /= 2;
   scalestate--;
}

static void
key(unsigned char k, int x, int y)
{
   struct trace *trace = traces + curtrace;
   int i;
   switch (k) {
   case '1': curtrace = 0; break;
   case '2': curtrace = 1; break;
   case '3': curtrace = 2; break;
   case '4': curtrace = 3; break;
   case '5': curtrace = 4; break;
   case '6': curtrace = 5; break;
   case '7': curtrace = 6; break;
   case '8': curtrace = 7; break;
   case '9': curtrace = 8; break;
   case '0': curtrace = 9; break;
   case '+': increasescale(); break;
   case '-': decreasescale(); break;
   case 'f':
      freeze = !freeze;
      break;
   case 'p':
      drawpoints = !drawpoints;
      break;
  case 'c': /* center */
     if(trace->datapoints)
        trace->offset = trace->datapoints->data;
     break;
   case 'C':
      for(i = 0; i<numtraces; i++)
         if(traces[i].datapoints)
            traces[i].offset = traces[i].datapoints->data;
      break;
   case 'v':
      trace->visible = !trace->visible;
      break;
   case 'V':
      for(i = 0; i<numtraces; i++)
         traces[i].visible = !traces[i].visible;
      break;
   case 'z': /* zero offset */
      trace->offset = 0;
      break;
   case 'Z':
      for(i = 0; i<numtraces; i++)
         traces[i].offset = 0;
      break;
   case 'w':
      ffton = !ffton;
      break;
  case 27:  /* Escape */
    exit(0);
    break;
  default:
    return;
  }
  glutPostRedisplay();
}

static void
special(int key, int x, int y)
{
   struct trace *trace = traces + curtrace;
   double dist = scale / 10;
   switch(key) {
   case GLUT_KEY_DOWN:
      trace->offset += dist;
      break;
   case GLUT_KEY_UP:
      trace->offset -= dist;
      break;
   }
}

void initinput(int argc, char **argv)
{
   const char usage[] =
      "Usage: glscope [OPTIONS]\n"
      "Options may include: \n"
      "-h, --help  print this message\n"
      "-f, --file [FILENAME] read input from a file instead of stdin\n"
      "    --fps [FPS] set frame rate, default 20\n"
      "-p, --prefix [PREFIX] only use lines that start with PREFIX discarding PREFIX\n"
      "-d, --delay [DELAY] delay in seconds before reading next line\n"
      "\t\t(useful for replaying data from a log file)\n"
      "-t, --time [SECONDS] width of screen in seconds (or time units provided in file)\n"
      "-s, --scale [SCALE] Initial scale, will pick closest \"nice\" value\n";

   const struct option long_options[] = {
      {"help", 0, 0, 'h'},
      {"file", 1, 0, 'f'},
      {"fps", 1, 0, 1},
      {"prefix", 1, 0, 'p'},
      {"delay", 0, 0, 'd'},
      {"time", 1, 0, 't'},
      {"scale", 1, 0, 's'},
      {0, 0, 0, 0}};

   char *filename = NULL;

   for(;;)
      switch(getopt_long(argc, argv, "hf:p:d:t:s:", long_options, NULL)) {
      case -1:
         goto nomoreargs;
      case 'h':
         fputs(usage, stdout);
         exit(0);
      case 'f':
         filename = malloc(strlen(optarg) + 1);
         strcpy(filename, optarg);
         break;
      case 1:
         fps = strtod(optarg, NULL);
         break;
      case 'p':
          strcpy(prefix, optarg);
          break;
      case 'd':
         delay = strtod(optarg, NULL);
         break;
      case 't':
         disptime = strtod(optarg, NULL);
         break;
      case 's':
         {
            double s = strtod(optarg, NULL);
            if(s <= 0)
               printf("Invalid scale: %g\n", s);
            else {
               while(s < scale) decreasescale();
               while(s > scale) increasescale();
            }
         }
      }

 nomoreargs:

   if(filename) {
      if(!(fd = open(filename, O_RDONLY))) {
         fprintf(stderr, "Failed to open '%s' for reading\n", filename);
         free(filename);
         exit(1);
      }
      free(filename);
   } else
      fd = 0;

   fcntl(fd, F_SETFL, O_NONBLOCK);

   starttime = gettime();
}

int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   initinput(argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);// | GLUT_ALPHA);
   glutCreateWindow ("glscope");
   glutFullScreen();
   initgl();
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutKeyboardFunc(key);
   glutSpecialFunc(special);
   glutIdleFunc(idle);
   inittraces();

   glutTimerFunc(0, timeout, 1000/fps);
   glutMainLoop();
   return 0;   /* ANSI C requires main to return int. */
}
