#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

namespace constants {
	constexpr double g{ 9.81 }; //ускорение свободного падения
	constexpr double air_density{ 1.225 }; //плотность воздуха в кг/м^3
	constexpr double pi{ 3.14159265358979323846 }; //число Пи

	constexpr double tyre_degradation_corner{ 0.0004 }; //износ шины в повороте
	constexpr double tyre_degradation_straight{ 0.0001 }; //износ шины на прямой

	constexpr double fuel_burn_corner{ 0.0015 }; //расход топлива в поворотах (кг/с)
	constexpr double fuel_burn_straight{ 0.004 }; //расход топлива на прямых (кг/с)

}

namespace temp_const {
	constexpr double weight{ 1450.0 }; //вес автомобиля в кг (1350 - вес машины, 100 - вес топлива)
	constexpr double grip_tyre_coef{ 1.5 }; //коэффицент сцепления шин
	constexpr double lift_coef{ 1.35 }; //коэффицент прижимной силы
	constexpr double front_area{ 2.2 }; //фронтальная площадь
	constexpr double drag_coef{ 0.5 }; //коэффицент лобового сопротивления

	constexpr double k_front{ 54.2 }; //коэффицент распределения веса на переднюю ось в процентах
	constexpr double k_rear{ 32.7 }; //коэффицент распределения веса на заднюю ось в процентах
}

struct Section {
	double lenght{}; //длина поворота
	bool is_corner{};
	double degree{}; //угол поворота

	double radian_degree{};
	double radius{};
};


double force_center(double radius, double velocity, double car_weight) { //радиус дан в метрах
	return car_weight * pow(velocity, 2.0) / radius;
}

double force_grip(double grip, double car_weight) {
	return grip * car_weight * constants::g;
}

double basic_limit_velocity(double radius, double grip) {
	return pow(grip * constants::g * radius, 0.5);
}

double downforce(double velocity) {
	return 0.5 * constants::air_density * pow(velocity, 2.0) * temp_const::lift_coef * temp_const::front_area;
}

double downforce_limit_velocity(double radius, double grip, double car_weight) {
	double numerator{grip * constants::g * radius};
	double denominator_1{ grip * constants::air_density * temp_const::lift_coef * temp_const::front_area * radius / (2 * car_weight)};

	return pow(numerator / (1 - denominator_1), 0.5);
}

double general_aero_coef(double grip, double car_weight) {
	double numerator{ constants::air_density * temp_const::front_area * (grip * temp_const::lift_coef + temp_const::drag_coef)};
	double denominator{ 2 * car_weight};

	return numerator / denominator;
}

double brake_dist(double current_velocity, double corner_limit_velocity, double grip, double car_weight) {
	double numerator{ (constants::g * grip) + (general_aero_coef(grip, car_weight) * pow(current_velocity, 2.0)) };
	double denominator{ (constants::g * grip) + (general_aero_coef(grip, car_weight) * pow(corner_limit_velocity, 2.0)) };

	double brake_distance{ (1.0 / (2 * general_aero_coef(grip, car_weight))) * std::log(numerator / denominator) };
	if (brake_distance < 0) return 0.0;
	return brake_distance;
}

double central_accel(double velocity, double radius) {
	if (radius <= 0) return 0.0;
	return pow(velocity, 2.0) / radius;
}

double limit_accel(double velocity, double grip, double car_weight) {
	return grip * constants::g +
		((grip * constants::air_density * pow(velocity, 2.0) * temp_const::lift_coef * temp_const::front_area) / (2 * car_weight));
}

double full_accel(double velocity, double radius, double grip, double car_weight) {
	double limit{ limit_accel(velocity, grip, car_weight) };
	double central{ central_accel(velocity, radius) };

	if (central < limit) {
		return pow(pow(limit, 2.0) - pow(central, 2.0), 0.5) -
			((limit / grip) - constants::g);
	}
	else return 0.0;
}

double time_straight_to_brake(double lenght, double current_velocity, double corner_limit_velocity, double& brake_distance, double grip, double car_weight) {
	double dt{ 0.001 };
	double total_time{ 0 };

	if (lenght < brake_dist(current_velocity, corner_limit_velocity, grip, car_weight)) {
		std::cout << "\nThe car flies out of the turn!\n";
		return -1;
	}

	while (lenght > brake_distance) {
		double accel = full_accel(current_velocity, 0.0, grip, car_weight);

		double temp_velocity{ current_velocity + accel * dt };
		lenght -= ((current_velocity * dt) + (0.5 * accel * pow(dt, 2.0)));
		total_time += dt;
		if (temp_velocity > 350 * 1000.0 / 3600.0) current_velocity = 350 * 1000.0 / 3600.0;
		else current_velocity = temp_velocity;
		if (accel <= 0 && current_velocity < 1.0) {
			break;
		}
		brake_distance = brake_dist(current_velocity, corner_limit_velocity, grip, car_weight);
	}

	return total_time;
}

double brake_time_to_corner(double current_velocity, double corner_speed, double brake_distance) {
	return (2 * brake_distance) / (current_velocity + corner_speed);
}

double time_inside_corner(double lenght, double& current_velocity, double base_radius, double grip, double car_weight) {
	double dt{ 0.001 };
	double total_time{ 0 };

	double distance_to_apex = lenght / 2.0;
	double safe_speed = downforce_limit_velocity(base_radius, grip, car_weight);

	total_time += distance_to_apex / safe_speed;
	current_velocity = safe_speed;

	double distance_left = lenght / 2.0;

	while (distance_left > 0) {
		double progress = 1.0 - (distance_left / (lenght / 2.0));

		double current_radius = base_radius / (1.0 - progress * 0.8);

		double accel = full_accel(current_velocity, current_radius, grip, car_weight);

		double temp_velocity = current_velocity + accel * dt;

		if (temp_velocity > 350.0 * 1000.0 / 3600.0) {
			current_velocity = 350.0 * 1000.0 / 3600.0;
		}
		else {
			current_velocity = temp_velocity;
		}

		distance_left -= (current_velocity * dt + 0.5 * accel * pow(dt, 2.0));
		total_time += dt;
	}

	return total_time;
}



int main() {

	double car_weight = temp_const::weight;
	double grip = temp_const::grip_tyre_coef;

	std::cout << "Enter number of track sections: ";
	std::size_t count;
	std::cin >> count;
	while (count < 1) {
		std::cout << "\nIncorrent value of corner (<1). Try again: ";
		std::cin >> count;
	}


	std::vector<Section> Track;

	for (std::size_t i{ 0 }; i < count; ++i) {
		Section temp{};
		std::cout << "\n\nIs a corner? (1/0) ";
		std::cin >> temp.is_corner;
		if (temp.is_corner) {
			std::cout << "Enter lenght of corner: ";
			std::cin >> temp.lenght;
			while (temp.lenght <= 0) {
				std::cout << "\nIncorrect enter! Lenght < 0. Try again: ";
				std::cin >> temp.lenght;
			}
			std::cout << "Enter degree of corner: ";
			std::cin >> temp.degree;
			while (temp.degree <= 0) {
				std::cout << "\nIncorrect enter! Degree < 0. Try again: ";
				std::cin >> temp.degree;
			}
			temp.radian_degree = temp.degree * constants::pi / 180;
			temp.radius = temp.lenght / temp.radian_degree;
		}
		else {
			std::cout << "Enter lenght of straight: ";
			std::cin >> temp.lenght;
			while (temp.lenght <= 0) {
				std::cout << "\nIncorrect enter! Lenght < 0. Try again: ";
				std::cin >> temp.lenght;
			}
			temp.degree = 0;
			temp.radius = 0;
		}
		std::cout << "Section #" << i + 1 << " information:"
			<< "\n	1. Is corner: " << temp.is_corner
			<< "\n	2. Lenght: " << temp.lenght << "m "
			<< "\n	3. Degree: " << temp.degree << "* "
			<< "\n	4. Radius: " << temp.radius << "m ";

		Track.push_back(temp);
	}

	double velocity{};
	std::cout << "\n\nEnter vehicle on start (km/h): ";
	std::cin >> velocity;
	velocity *= 1000.0 / 3600.0; // переводим в м/с

	std::cout << "\nEnter value of laps: ";
	int lap{};
	std::cin >> lap;
	while (lap < 1) {
		std::cout << "Incorrect enter! Lap < 1";
		std::cin >> lap;
	}
	double total_time{ 0 };

	std::cout << "\n==============RACE==============\n";

	for (int i{ 0 }; i < lap; ++i) {
		double lap_time{ 0 };
		for (std::size_t j{ 0 }; j < count; ++j) {

			std::size_t next_index = (j + 1) % count;
			double radius_next_corner{ Track[next_index].radius };
			double max_speed_next_corner{ downforce_limit_velocity(radius_next_corner, grip, car_weight) };

			double brake_distance{ brake_dist(velocity, max_speed_next_corner, grip, car_weight) };
			double lenght{ Track[j].lenght };
			double section_time{ 0 };

			if (!Track[j].is_corner) {
				section_time = time_straight_to_brake(lenght, velocity, max_speed_next_corner, brake_distance, grip, car_weight);
				section_time += brake_time_to_corner(velocity, max_speed_next_corner, brake_distance);
				velocity = max_speed_next_corner;
				grip -= constants::tyre_degradation_straight;
				car_weight -= constants::fuel_burn_straight;
			}
			else {
				section_time = time_inside_corner(lenght, velocity, Track[j].radius, grip, car_weight);
				grip -= constants::tyre_degradation_corner;
				car_weight -= constants::fuel_burn_corner;
			}
			lap_time += section_time;
		}

		int minutes_lap_time{ static_cast<int>(lap_time / 60) };
		double seconds_lap_time{ lap_time - minutes_lap_time * 60 };


		std::cout << "Lap " << i + 1 << " time: " << minutes_lap_time << ":" << seconds_lap_time << "s\n";
		total_time += lap_time;
	}

	std::cout << "\n\nTotal time for all laps: " << total_time << "s \n";

	/*double front_brake_balance{};
	std::cout << "Enter front brake balance (0 - 1.0): ";
	std::cin >> front_brake_balance;
	while (front_brake_balance > 1 || front_brake_balance < 0) {
		std::cout << "Invalid input. Please enter a value between 0 and 1: ";
		std::cin >> front_brake_balance;
	}
	double rear_brake_balance{ 1.0 - front_brake_balance };

	std::cout << "\nBrake balance (front|rear): " << front_brake_balance * 100 << "% | " << rear_brake_balance * 100 << "%\n";

	std::cout << "\nEnter force on brake pedal: ";
	double pedal_force{};
	std::cin >> pedal_force;

	std::cout << "\nAerodynamics: " << downforce_limit_velocity(100) << " m/s  |  No aerodynamics: " << basic_limit_velocity(100) << " m/s\n";
	std::cout << "Aerodynamics: " << downforce_limit_velocity(100) * 3.6 << " km/h  |  No aerodynamics: " << basic_limit_velocity(100) * 3.6 << " km/h\n";
	double force_brake_front{ temp_const::k_front * front_brake_balance * pedal_force };
	double force_brake_rear{ temp_const::k_rear * rear_brake_balance * pedal_force };

	double total_brake_force{ force_brake_front + force_brake_rear };

	std::cout << "\nFront brake force: " << force_brake_front << " N\n"
			  << "Rear brake force: " << force_brake_rear << " N\n"
			  << "Total brake force: " << total_brake_force << " N\n";

	std::cout << "\nMaximum possible braking force: " << std::min(grip * (car_weight * constants::g + downforce(velocity)), total_brake_force) << " N\n";

	std::cout << "\nEnter radius next corner: ";
	double radius_corner{};
	std::cin >> radius_corner;

	*/



	return 0;
}
