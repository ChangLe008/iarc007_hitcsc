#include "iarc007/hy_target/angleID.h"
#include <math.h>

void angleID::initialize(){

	max = 20;
	num = 0;
	last_time = ros::Time::now();
	current_time = ros::Time::now();
	r_min = 0.1;
	duration = 1.0;

	no_num = 0;

	mem.resize(max);
	angle = 10.;

	update_flag = false;
	current_time_ir = last_time_ir = ros::Time::now();

	positive_ang = 0.;
	negative_ang = 0.;

	missing_time = 2.;

	positive_num = 0;
	negative_num = 0;
	missing_num = missing_num_max = 15;
	missing_num_min = 10;
}

void angleID::update(float air_pos[], Point2f point, bool flag) //to get measure data
{
	this->current_time = ros::Time::now();

	//if(fabs(double(current_time.toSec())-double(last_time.toSec())) > duration){
	if (no_num > 8){
		angleID::memClear();
		this->valid = false;

		no_num = 0;

		return;
	}
	else{
		if (!flag){
			//TODO delete
			if (this->num > 0){
				this->num -= 1;
			}
			this->valid = false;

			no_num++;

			return;
		}
		else{
			if (no_num > 0){
				no_num--;
			}

			if (this->num < this->max){
				//TODO insert

				this->mem.at(this->num).x = air_pos[0] + point.x;
				this->mem.at(this->num).y = air_pos[1] + point.y;
				this->num++;

				this->valid = false;
			}
			else{
				//TODO insert and delete
				for (int i = 0; i < max - 1; i++){
					this->mem.at(i).x = this->mem.at(i + 1).x;
					this->mem.at(i).y = this->mem.at(i + 1).y;
				}

				this->mem.at(this->max - 1).x = air_pos[0] + point.x;
				this->mem.at(this->max - 1).y = air_pos[1] + point.y;

				valid = true;
			}
		}
	}
}

bool angleID::estimate()
{
	float a = 0., b = 0.;
	float r = 0.;
	//cout << "---------------valid= " << valid << endl;
	if (valid){
		float sum_xx = sumPow(this->mem);
		float sum_xy = sumMultiply(this->mem);
		float sum_x = sum(this->mem, true);
		float sum_y = sum(this->mem, false);

		a = (sum_xx*sum_y - sum_x*sum_xy) / (max*sum_xx - sum_x*sum_x);
		b = (max*sum_xy - sum_x*sum_y) / (max*sum_xx - sum_x*sum_x);

		float sum_diff_x = sumDiff(this->mem, true);
		float sum_diff_y = sumDiff(this->mem, false);

		float sum_diff_x2 = sumDiff2(this->mem, true);
		float sum_diff_y2 = sumDiff2(this->mem, false);

		r = (sum_diff_x*sum_diff_y) / (sqrt(sum_diff_x2)*sqrt(sum_diff_y2));

		//cout << "--------relevant: " << fabs(r) << endl;		

		//if(fabs(r) < this->r_min){
		if (0){

			this->angle = atan(a);

			cout << "-----------angle: " << this->angle << endl;

			float front_x = 0., behind_x = 0.;
			float front_y = 0., behind_y = 0.;

			if (this->angle < PI / 4. || this->angle > 1.5*PI){

				for (int i = 0, front_x = 0.; i < this->max / 2; i++){
					front_x += this->mem.at(i).x;
				}

				for (int i = this->max / 2, behind_x = 0.; i < this->max; i++){
					behind_x += this->mem.at(i).x;
				}

				if (behind_x > front_x){
					if (this->angle > PI / 2.){
						this->angle = this->angle - PI;
					}
				}
				else{
					if (this->angle < PI / 2.){
						this->angle = this->angle - PI;
					}
				}
			}
			else{
				for (int i = 0, front_y = 0.; i < this->max / 2; i++){
					front_y += this->mem.at(i).y;
				}

				for (int i = this->max / 2, behind_y = 0.; i < this->max; i++){
					behind_y += this->mem.at(i).y;
				}

				if (behind_y < front_y){
					this->angle = this->angle - PI;
				}
			}

			this->last_time = this->current_time;

			return true;
		}
		else{
			return false;
		}
	}
}


void angleID::memClear()
{
	this->last_time = this->current_time;
	this->num = 0;
}

float angleID::sumPow(std::vector<Point2f>& mem)
{
	float sum_xx = 0.;

	for (int i = 0; i < this->max; i++)
	{
		sum_xx += mem.at(i).x*mem.at(i).x;
	}

	return sum_xx;
}

float angleID::sumMultiply(std::vector<Point2f>& mem)
{
	float sum_xy = 0.;

	for (int i = 0; i < this->max; i++)
	{
		sum_xy += mem.at(i).x*mem.at(i).y;
	}

	return sum_xy;
}

float angleID::sum(std::vector<Point2f>& mem, bool flag)
{
	float sum = 0.;

	if (flag){
		for (int i = 0; i < this->max; i++){
			sum += mem.at(i).x;
		}
	}
	else{
		for (int i = 0; i < this->max; i++){
			sum += mem.at(i).y;
		}
	}

	return sum;
}

float angleID::avg(std::vector<Point2f>& mem, bool flag)
{
	float avg = 0.;

	avg = angleID::sum(mem, flag) / this->max;

	return avg;
}

float angleID::sumDiff(std::vector<Point2f>& mem, bool flag)
{
	float sum_diff = 0.;

	float avg = angleID::avg(mem, flag);

	if (flag){
		for (int i = 0; i < this->max; i++){
			sum_diff += (mem.at(i).x - avg);
		}
	}
	else{
		for (int i = 0; i < this->max; i++){
			sum_diff += (mem.at(i).y - avg);
		}
	}

	return sum_diff;
}

float angleID::sumDiff2(std::vector<Point2f>& mem, bool flag)
{
	float sum_diff2 = 0.;

	float avg = angleID::avg(mem, flag);

	if (flag){
		for (int i = 0; i < this->max; i++){
			sum_diff2 += (mem[i].x - avg)*(mem[i].x - avg);
		}
	}
	else{
		for (int i = 0; i < this->max; i++){
			sum_diff2 += (mem.at(i).y - avg)*(mem.at(i).y - avg);
		}
	}

	return sum_diff2;
}

bool angleID::filter(Point2f& point, float& theta, bool flag)
{
	current_time_ir = ros::Time::now();

	//if(((double)current_time_ir.toSec() - (double)(last_time_ir.toSec())) > missing_time){
	if (0){

		cout << "-----------Out of missing_time----------- :" << ((double)current_time_ir.toSec() - (double)(last_time_ir.toSec())) << endl;
		return false;
	}
	else{

		if (missing_num < missing_num_min){
			if (flag){
				update_flag = true;

				if (missing_num > 0){
					missing_num--;
				}

				if (positive_num > negative_num){
					if (fabs(positive_ang - theta) < 1.0){
						positive_ang = theta = 0.8*theta + 0.2*positive_ang;

						if (positive_num < 20){
							positive_num++;
						}
						else{
							positive_num = 20;
						}

						if (negative_num > 0){
							negative_num--;
						}
					}
					else{
						theta = positive_ang;

						if (negative_num < 20){
							negative_num++;
						}
						else{
							negative_num = 20;
						}

						if (positive_num > 0){
							positive_num--;
						}
					}
				}
				else{
					if (fabs(negative_ang - theta) < 1.0){
						negative_ang = theta = 0.8*theta + 0.2*negative_ang;

						if (negative_num < 20){
							negative_num++;
						}
						else{
							negative_num = 20;
						}
						if (positive_num > 0){
							positive_num--;
						}
					}
					else{
						theta = negative_ang;

						if (positive_num < 20){
							positive_num++;
						}
						else{
							positive_num = 20;
						}

						if (negative_num > 0){
							negative_num--;
						}
					}
				}
				//TODO
				/*if(fabs(theta-positive_ang)>fabs(theta-negative_ang)){
				if(positive_num > 0){
				positive_num++;
				}
				negative_num--;
				positive_ang = theta = 0.8*positive_ang
				}
				else{
				positive_num++;
				if(negative_num > 0){
				negative_num--;
				}
				}*/

				point.x = 0.9*point.x + 0.1*target.x;
				point.y = 0.9*point.y + 0.1*target.y;

				target = point;

				return true;
			}
			else{
				missing_num++;
				positive_num > negative_num ? theta = positive_ang : theta = negative_ang;
				point = target;

				if (negative_num > 0){
					negative_num--;
				}
				if (positive_num > 0){
					positive_num--;
				}

				update_flag = true;

				return true;
			}
		}
		else{
			if (!flag){
				if (update_flag){
					last_time_ir = current_time_ir;
					update_flag = false;

					//missing_num = 0;

					if (positive_num > negative_num){
						//theta = negative_ang;
						theta = positive_ang;
						point = target;

						return true;
					}
					else{
						//theta = positive_ang;
						theta = negative_ang;
						point = target;
						return true;
					}
				}
				else{
					if (missing_num < missing_num_max){
						missing_num++;

						positive_num > negative_num ? theta = positive_ang : theta = negative_ang;
						point = target;
						return true;
					}
					else{
						positive_num = 0;
						negative_num = 0;

						return false;
					}

					if (positive_num > 0){
						positive_num--;
					}
					if (negative_num > 0){
						negative_num--;
					}
				}
			}
			else{
				if (missing_num < missing_num_max){
					point.x = 0.9*point.x + 0.1*target.x;
					point.y = 0.9*point.y + 0.1*target.y;
					target = point;
				}
				else{
					target = point;
				}

				missing_num--;

				if (fabs(positive_ang - theta) < 0.5){
					if (positive_num < 20){
						positive_num++;
					}
					else{
						positive_num = 20;
					}

					positive_ang = 0.1*positive_ang + 0.9*theta;
					theta = positive_ang;
				}
				else if (fabs(negative_ang - theta) < 0.5){
					if (negative_num < 20){
						negative_num++;
					}
					else{
						negative_num = 20;
					}
					negative_ang = 0.2*negative_ang + 0.8*theta;
					theta = negative_ang;
				}
				else{
					positive_num = negative_num = 1;
					positive_ang = theta;
					if (positive_ang > 0.){
						negative_ang = theta - PI;
					}
					else{
						negative_ang = theta + PI;
					}
				}

				update_flag = true;

				return false;
			}
		}
	}
}
