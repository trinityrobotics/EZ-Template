/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

namespace ez {
int mode = DISABLE;

void print_ez_template() {
  std::cout << R"(


    _____ ______   _____                    _       _
   |  ___|___  /  |_   _|                  | |     | |
   | |__    / /_____| | ___ _ __ ___  _ __ | | __ _| |_ ___
   |  __|  / /______| |/ _ \ '_ ` _ \| '_ \| |/ _` | __/ _ \
   | |___./ /___    | |  __/ | | | | | |_) | | (_| | ||  __/
   \____/\_____/    \_/\___|_| |_| |_| .__/|_|\__,_|\__\___|
                                     | |
                                     |_|
)" << '\n';

  printf("Version: 2.1.1\n");
}
std::string get_last_word(std::string text) {
  std::string word = "";
  for (int i = text.length() - 1; i >= 0; i--) {
    if (text[i] != ' ') {
      word += text[i];
    } else {
      std::reverse(word.begin(), word.end());
      return word;
    }
  }
  std::reverse(word.begin(), word.end());
  return word;
}
std::string get_rest_of_the_word(std::string text, int position) {
  std::string word = "";
  for (int i = position; i < text.length(); i++) {
    if (text[i] != ' ' && text[i] != '\n') {
      word += text[i];
    } else {
      return word;
    }
  }
  return word;
}
//All iance\n\nWE WIN THESE!!!!! 
void print_to_screen(std::string text, int line) {
  int CurrAutoLine = line;
  std::vector<string> texts = {};
  std::string temp = "";

  for (int i = 0; i < text.length(); i++) {
    if (text[i] != '\n' && temp.length() + 1 > 32) {
      auto last_word = get_last_word(temp);
      if (last_word == temp) {
        texts.push_back(temp);
        temp = text[i];
      } else {
        int size = last_word.length(); 

        auto rest_of_word = get_rest_of_the_word(text, i); 
        temp.erase(temp.length() - size, size);
        texts.push_back(temp);
        last_word += rest_of_word;
        i += rest_of_word.length();
        temp = last_word;
        if (i >= text.length() - 1) {
          texts.push_back(temp);
          break;
        }
        
      }
    }
    if (i >= text.length() - 1) {
      temp += text[i];
      texts.push_back(temp);
      temp = "";
      break;
    } else if (text[i] == '\n') {
      texts.push_back(temp);
      temp = "";
    } else {
      temp += text[i];
    }
  }
  for (auto i : texts) {
    if (CurrAutoLine > 7) {
      pros::lcd::clear();
      pros::lcd::set_text(line, "Out of Bounds. Print Line is too far down");
      return;
    }
    pros::lcd::clear_line(CurrAutoLine);
    pros::lcd::set_text(CurrAutoLine, i);
    CurrAutoLine++;
  }
}

std::string exit_to_string(exit_output input) {
  switch ((int)input) {
    case RUNNING:
      return "Running";
    case SMALL_EXIT:
      return "Small";
    case BIG_EXIT:
      return "Big";
    case VELOCITY_EXIT:
      return "Velocity";
    case mA_EXIT:
      return "mA";
    case ERROR_NO_CONSTANTS:
      return "Error: Exit condition constants not set!";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}
namespace util {
bool AUTON_RAN = true;

bool is_reversed(double input) {
  if (input < 0) return true;
  return false;
}

int sgn(double input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

double clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}

double get_angle(double x1, double y1, double x2, double y2) {
  //
  // Use atan2 to get an angle between two points 
  //
  // double yDiff = (y2 - y1);
  // double y = cos(x2) * sin(yDiff);
  // double x = cos(x1) * sin(x2) - sin(x1) * cos(x2) * cos(yDiff);
  // This is a bearing off of 0 degrees (negative mean counter clockwise)
  // double degrees = atan2(y, x)/M_PI*180;

  double x = x2 - x1;
  double y = y2 - y1;
    // theta = math.atan2(dy, dx)
    // angle = math.degrees(theta)  // angle is in (-180, 180]
    // if angle < 0:
    //     angle = 360 + angle
    // return angle
  double theta_rad = atan2(y,x);
  double theta_deg = (theta_rad/M_PI*180); // + (theta_rad > 0 ? 0 : 360);
  if (theta_deg > 90)
  {
    theta_deg = 450 - theta_deg;
  } else {
    theta_deg = 90 - theta_deg;
  }
  return theta_deg;
}

double corrected_heading(double original_heading, double gps_offset) {
  double corrected_heading = original_heading - gps_offset;
  if (corrected_heading < 0) {
    return corrected_heading + 360;
  } else {
    return corrected_heading;
  }
}

double get_distance(double x1, double y1, double x2, double y2, double offset) {
  //
  // Return the hypotenuse
  //
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0) - offset;
}

double m_to_in(double meters) {
  //
  // Return the hypotenuse
  //
  return meters * 39.37;
}

double in_to_m(double inches) {
  //
  // Return the hypotenuse
  //
  return inches / 39.37;
}

}  // namespace util
}  // namespace ez
