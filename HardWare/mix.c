#include "mix.h"
#include "math.h"
#include "MS5607.h"

#define Kp 10.0f
#define Ki 0.53f
#define halfT 0.001f

extern quaternion attitude;
Vector3f acc,acc_ef;
Vector3f accel_correction_ef,_velocity,_position_correction,_position_base;
Vector3f _position_error;
Location current_loc;
Matrix3f Rotate_Matrix;

float Pitch,Roll,Yaw,Pitchb,Rollb,Pitchb_J,Rollb_J;
extern  float Acc[3],Gyro[3];


const static float MIX_MAG_Y = 0.9396926f;
const static float MIX_MAG_Z = -0.342020143f;

{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;                       // evil floating point bit level hacking
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

    return y;
}

void quaternion_loadIdentity(quaternion * q)
{
    q->w = 1;
    q->x = q->y = q->z = 0;
}


void mix_gyrAcc_crossMethod(const float gyr[3],const float acc[3])
{

	float norm, delta_2=0;
	float vx, vy, vz, ex, ey, ez;
	static float qq[M] = {1,0,0,0};   
	float exInt = 0, eyInt = 0, ezInt = 0; 

	float q0q0 = qq[0]*qq[0];
	float q0q1 = qq[0]*qq[1];
	float q0q2 = qq[0]*qq[2];
	float q1q1 = qq[1]*qq[1];
	float q1q3 = qq[1]*qq[3];
	float q2q2 = qq[2]*qq[2];
	float q2q3 = qq[2]*qq[3];
	float q3q3 = qq[3]*qq[3];
	float ax=acc[0],ay=acc[1],az=acc[2];
	float gx=gyr[0],gy=gyr[1],gz=gyr[2];
		
	if(ax*ay*az==0)
		return;
			
	norm = sqrt(ax*ax + ay*ay + az*az);      
	ax = ax /norm;
	ay = ay / norm;
	az = az / norm;


	vx = 2*(q1q3 - q0q2);	
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	ex = (ay*vz - az*vy) ;       
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
		
	exInt = exInt + ex * Ki*halfT;	
	eyInt = eyInt + ey * Ki*halfT;
	ezInt = ezInt + ez * Ki*halfT;

	gx = gx + Kp*ex + exInt;	
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;	
		 	
	
	delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);	
	qq[0] = (1-delta_2/8)*qq[0] + (-qq[1]*gx - qq[2]*gy - qq[3]*gz)*halfT;		
	qq[1] = (1-delta_2/8)*qq[1] + ( qq[0]*gx + qq[2]*gz - qq[3]*gy)*halfT;
	qq[2] = (1-delta_2/8)*qq[2] + ( qq[0]*gy - qq[1]*gz + qq[3]*gx)*halfT;
	qq[3] = (1-delta_2/8)*qq[3] + ( qq[0]*gz + qq[1]*gy - qq[2]*gx)*halfT;	

	norm = sqrt(qq[0]*qq[0] + qq[1]*qq[1] + qq[2]*qq[2] + qq[3]*qq[3]);
	qq[0] = qq[0] / norm;
	qq[1] = qq[1] / norm;
	qq[2] = qq[2] / norm;
	qq[3] = qq[3] / norm;

	Pitch = angle_pitch(qq)-4.75; 
	Roll =  angle_roll(qq)+2.32; 
	Yaw= angle_yaw(qq);
}


Vector3f get_accel_ef(Matrix3f * Mat,Vector3f*T)
{
	Vector3f Te;
	Te.x = T->x*Mat->a.x + T->y*Mat->a.y + T->z*Mat->a.z;
	Te.y = T->x*Mat->b.x + T->y*Mat->b.y + T->z*Mat->b.z;
	Te.z = T->x*Mat->c.x + T->y*Mat->c.y + T->z*Mat->c.z;
		
	return Te;
}


signed int SENGJIANG, SENGJIANGX, SENGJIANGY;

Vector3f velocity_increase;
float accccz;
void InertialNav_update(float dt)
{
   
	float tmp,ftmpx,ftmpy; 	
	int temp;

    if( dt > 0.1f ) {
        return;
    }

	acc.x = Acc[0];
	acc.y = Acc[1];
	acc.z = Acc[2];
		
    acc_ef = get_accel_ef(&Rotate_Matrix,&acc);
		
	ftmpx= acc_ef.x;
	ftmpy= acc_ef.y;
		
	acc_ef.x=ftmpx*cos(Yaw)+ftmpy*sin(Yaw);
	acc_ef.y=ftmpy*cos(Yaw)-ftmpx*sin(Yaw);

	acc_ef.z -= GRAVITY_MSS;

    acc_ef.x *= 100;
 	acc_ef.y *= 100;
	acc_ef.z *= 100;

    tmp = _k3_xy * dt;
    accel_correction_ef.x += _position_error.x * tmp;
    accel_correction_ef.y += _position_error.y * tmp;
    accel_correction_ef.z += _position_error.z * _k3_z  * dt;
 
    tmp = _k2_xy * dt;
    _velocity.x += _position_error.x * tmp;
    _velocity.y += _position_error.y * tmp;
    _velocity.z += _position_error.z * _k2_z  * dt;

    tmp = _k1_xy * dt;
    _position_correction.x += _position_error.x * tmp;
    _position_correction.y += _position_error.y * tmp;
    _position_correction.z += _position_error.z * _k1_z  * dt;

    
    velocity_increase.x = (acc_ef.x + accel_correction_ef.x) * dt;
 	velocity_increase.y = (acc_ef.y + accel_correction_ef.y) * dt;
	velocity_increase.z = (acc_ef.z + accel_correction_ef.z) * dt;

   
    _position_base.x += (_velocity.x + velocity_increase.x*0.5) * dt;
 	_position_base.y += (_velocity.y + velocity_increase.y*0.5) * dt;
	_position_base.z += (_velocity.z + velocity_increase.z*0.5) * dt;
	
    
    _velocity.x += velocity_increase.x;
 	_velocity.y += velocity_increase.y;
	_velocity.z += velocity_increase.z;
	
	add(_position_base.x,0);
    add(_position_base.y,1);
    add(_position_base.z,2);
    current_loc.alt= (int)(_position_base.z + _position_correction.z);	   
    current_loc.lat= (int)(_position_base.x + _position_correction.x);	   
    current_loc.lng= (int)(_position_base.y + _position_correction.y);
}

#define FILTER_SIZE  7
float          samples[FILTER_SIZE];   
unsigned  int  _timestamps[FILTER_SIZE];
unsigned char  _new_data;
float          _last_slope;
unsigned char  sample_index; 

// apply - take in a new raw sample, and return the filtered results
float apply(float sample)
{
    // add sample to array
    samples[sample_index++] = sample;

    // wrap index if necessary
    if( sample_index >= FILTER_SIZE )
        sample_index = 0;

    // base class doesn't know what filtering to do so we just return the raw sample
    return sample;
}


void  update(float sample, unsigned  int timestamp)
{
    unsigned char i = sample_index;
    unsigned char i1;
    if (i == 0) {
        i1 = FILTER_SIZE-1;
    } else {
        i1 = i-1;
    }
    if (_timestamps[i1] == timestamp) {
        // this is not a new timestamp - ignore
        return;
    }

    // add timestamp before we apply to FilterWithBuffer
    _timestamps[i] = timestamp;

    // call parent's apply function to get the sample into the array
    apply(sample);

    _new_data = 1;
}


// use f() to make the code match the maths a bit better. Note
// that unlike an average filter, we care about the order of the elements
// #define f(i) samples[(((sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE]
float  f(signed char i )
{
	unsigned char b=0;
	b=sample_index-1+i+1;
	// +3*FILTER_SIZE/2) % FILTER_SIZE
	b+=10;
	b%=7;
	return  samples[b];
}
				
// #define x(i) _timestamps[(((sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE]
			
unsigned  int  x(signed char i )
{
	unsigned char b=0;
	b=sample_index-1+i+1;
	// +3*FILTER_SIZE/2) % FILTER_SIZE
	b+=10;
	b%=7;
	return _timestamps[b];
}			
			

float slope(void)
{
    unsigned char i;


	float result = 0,a=0,b=0,c=0,a1=0,b1=0,c1=0 ;


	if (!_new_data) {
        return _last_slope;
    }


    if (_timestamps[FILTER_SIZE-1] == _timestamps[FILTER_SIZE-2]) {
        // we haven't filled the buffer yet - assume zero derivative
        return 0;
    }

    // N in the paper is FILTER_SIZE
    switch (FILTER_SIZE) {
    case 5:
        result = 2*2*(f(1) - f(-1)) / (x(1) - x(-1))
                 + 4*1*(f(2) - f(-2)) / (x(2) - x(-2));
        result /= 8;
        break;
    case 7:
       /* result = 2*5*(f(1) - f(-1)) / (x(1) - x(-1))
                 + 4*4*(f(2) - f(-2)) / (x(2) - x(-2))
                 + 6*1*(f(3) - f(-3)) / (x(3) - x(-3));*/
		
		a=f(1);
		b=f(-1);
		c=a-b;
		c*=10;
        a=x(1);
		b=x(-1);	
		a=(a-b)/100;
		c=c/a;
		a1=c;
		
		a=f(2);
		b=f(-2);
		c=a-b;
		c*=16;
        a=x(2);
		b=x(-2);	
		a=(a-b)/1000;
		c=c/a;
		b1=c;
								
		a=f(3);
		b=f(-3);
		c=a-b;
		c*=6;
        a=x(3);
		b=x(-3);	
		a=(a-b)/1000;
		c=c/a;
		c1=c;
								 
		result=a1+b1+c1;
        result /= 32;
        break;
    case 9:
        result = 2*14*(f(1) - f(-1)) / (x(1) - x(-1))
                 + 4*14*(f(2) - f(-2)) / (x(2) - x(-2))
                 + 6* 6*(f(3) - f(-3)) / (x(3) - x(-3))
                 + 8* 1*(f(4) - f(-4)) / (x(4) - x(-4));
        result /= 128;
        break;
    case 11:
        result =  2*42*(f(1) - f(-1)) / (x(1) - x(-1))
                 +  4*48*(f(2) - f(-2)) / (x(2) - x(-2))
                 +  6*27*(f(3) - f(-3)) / (x(3) - x(-3))
                 +  8* 8*(f(4) - f(-4)) / (x(4) - x(-4))
                 + 10* 1*(f(5) - f(-5)) / (x(5) - x(-5));
        result /= 512;
        break;
    default:
        result = 0;
        break;
    }

// cope with numerical errors
// if (isnan(result) || isinf(result)) {
// 		result = 0;
//  }

	_new_data = 0;
	_last_slope = result;

    return result;
}


float angle_pitch(float b[M])
{
	float t31=(b[1]*b[3]-b[0]*b[2])*2.0;
	float pitch = -asin(t31)*57.3;
	return pitch;
}

float angle_roll(float c[M])
{
	float t32 = (c[2]*c[3]+c[0]*c[1])*2.0;
	float t33=c[0]*c[0]-c[1]*c[1]-c[2]*c[2]+c[3]*c[3];
	float roll= -atan2(t32,t33)*57.3;
	return roll;
}

float	angle_yaw(float	a[M])
{
	float t21=(a[1]*a[2]+a[0]*a[3])*2.0;
	float t11=a[0]*a[0]+a[1]*a[1]-a[2]*a[2]-a[3]*a[3];
	float yaw = atan2(t21,t11)*57.3;
	if(yaw<0) 
		yaw += 360.0f;
	return yaw;
}


