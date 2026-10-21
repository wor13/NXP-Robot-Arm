/* Portable reference primitives. Transport, feedback, stop and timing belong to
   the MCXN947 firmware; this file alone does not operate the arm. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
typedef struct { int32_t raw[6]; } PlexusPose;
typedef struct { PlexusPose pose; uint32_t dt_us; } PlexusTimedPose;
typedef struct { uint8_t kind; uint16_t pose_index; uint32_t minimum_ms,dwell_ms; } PlexusStep;
static inline bool plexus_in_limits(const PlexusPose *p,const int32_t lo[6],const int32_t hi[6]) {
 for(unsigned i=0;i<6;i++) if(p->raw[i]<lo[i] || p->raw[i]>hi[i]) return false;
 return true;
}
static inline PlexusPose plexus_merge(uint8_t kind,PlexusPose current,PlexusPose taught) {
 for(unsigned i=0;i<6;i++) if((kind==0 && i<5)||(kind==1 && i==5)) current.raw[i]=taught.raw[i];
 return current;
}
static inline PlexusPose plexus_interpolate(PlexusPose start,PlexusPose goal,uint32_t elapsed_ms,uint32_t duration_ms) {
 double u=duration_ms ? fmin(1.,(double)elapsed_ms/duration_ms) : 1.;
 double s=u*u*u*(10.+u*(-15.+6.*u));PlexusPose out;
 for(unsigned i=0;i<6;i++) out.raw[i]=(int32_t)nearbyint((double)start.raw[i]+((double)goal.raw[i]-start.raw[i])*s);
 return out;
}
static inline uint32_t plexus_duration_ms(PlexusPose start,PlexusPose goal,double ticks_per_s,double ticks_per_s2,uint32_t minimum_ms) {
 double distance=0.;for(unsigned i=0;i<6;i++) distance=fmax(distance,fabs((double)goal.raw[i]-start.raw[i]));
 double t=fmax(minimum_ms/1000.,fmax(1.875*distance/ticks_per_s,sqrt(5.774*distance/ticks_per_s2)));
 return (uint32_t)ceil(t*1000.);
}
