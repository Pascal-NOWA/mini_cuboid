#pragma once
#include <cstdint>
#include <map>
#include <vector>
#include "minicube_parametermap.h"

enum {
	FLAT = 1,
	BALANCE = 2,
	WIGGLE_SLOW = 3,
	WIGGLE_FAST = 4,
	WALK_LEFT = 5,
	WALK_RIGHT = 6,
    WALK_LEFT_FAST = 7,
	WALK_RIGHT_FAST = 8,
};

static std::map<uint8_t, std::vector<uint8_t>> choreography_map = {
	{RED_1, {1,2,2,2,2,2,2,2,3,3,2,2,3,3,2,2,2,2,4,4,2,2,4,4,5,5,4,4,3,3,3,4,6,6,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,5,5,4,4,4,4,6,6,4,4,6,6,5,5,4,4,3,2,3,2,3,2,3,1,77,}},
	{BLACK_1, {1,2,2,2,2,2,2,2,3,3,2,2,3,3,2,2,2,2,4,4,2,2,4,4,3,3,3,3,5,5,4,4,6,6,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,5,5,4,4,4,4,6,6,4,4,6,6,5,5,4,4,2,3,2,3,2,3,2,1,77,}},
	{BLUE_1, {1,2,2,2,2,2,2,2,3,3,2,2,3,3,2,2,2,2,4,4,2,2,4,4,3,3,3,3,6,6,4,4,5,5,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,4,4,5,5,6,6,4,4,6,6,4,4,4,4,5,5,3,2,3,2,3,2,3,1,77,}},
	{ORANGE_1, {1,2,2,2,2,2,2,2,3,3,2,2,3,3,2,2,2,2,4,4,2,2,4,4,6,6,4,4,3,3,3,4,5,5,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,4,4,5,5,6,6,4,4,6,6,4,4,4,4,5,5,2,3,2,3,2,3,2,1,77,}},
	{RED_2, {1,2,2,2,2,2,2,2,2,2,4,4,2,2,4,4,3,3,2,2,3,3,2,2,5,5,4,4,3,3,3,4,6,6,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,5,5,4,4,4,4,6,6,4,4,6,6,5,5,4,4,3,2,3,2,3,2,3,1,77,}},
	{BLACK_2, {1,2,2,2,2,2,2,2,2,2,4,4,2,2,4,4,3,3,2,2,3,3,2,2,3,3,3,3,5,5,4,4,6,6,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,5,5,4,4,4,4,6,6,4,4,6,6,5,5,4,4,2,3,2,3,2,3,2,1,77,}},
	{BLUE_2, {1,2,2,2,2,2,2,2,2,2,4,4,2,2,4,4,3,3,2,2,3,3,2,2,3,3,3,3,6,6,4,4,5,5,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,4,4,5,5,6,6,4,4,6,6,4,4,4,4,5,5,3,2,3,2,3,2,3,1,77,}},
	{ORANGE_2, {1,2,2,2,2,2,2,2,2,2,4,4,2,2,4,4,3,3,2,2,3,3,2,2,6,6,4,4,3,3,3,4,5,5,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,4,4,5,5,6,6,4,4,6,6,4,4,4,4,5,5,2,3,2,3,2,3,2,1,77,}},
	{RED_3, {1,2,2,2,2,2,2,2,3,3,2,2,3,3,2,2,2,2,4,4,2,2,4,4,5,2,4,4,3,3,3,4,6,6,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,5,5,4,4,4,4,6,6,4,4,6,6,5,5,4,4,3,2,3,2,3,2,3,1,77,}},
	{BLACK_3, {1,2,2,2,2,2,2,2,3,3,2,2,3,3,2,2,2,2,4,4,2,2,4,4,3,3,3,3,5,5,4,4,6,6,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,5,5,4,4,4,4,6,6,4,4,6,6,5,5,4,4,2,3,2,3,2,3,2,1,77,}},
	{BLUE_3, {1,2,2,2,2,2,2,2,3,3,2,2,3,3,2,2,2,2,4,4,2,2,4,4,3,3,3,3,6,6,4,4,5,5,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,4,4,5,5,6,6,4,4,6,6,4,4,4,4,5,5,3,2,3,2,3,2,3,1,77,}},
	{ORANGE_3, {1,2,2,2,2,2,2,2,3,3,2,2,3,3,2,2,2,2,4,4,2,2,4,4,6,6,4,4,3,3,3,4,5,5,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,4,4,5,5,6,6,4,4,6,6,4,4,4,4,5,5,2,3,2,3,2,3,2,1,77,}},
	{RED_4, {1,2,2,2,2,2,2,2,2,2,4,4,2,2,4,4,3,3,2,2,3,3,2,2,5,5,4,4,3,3,3,4,6,6,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,5,5,4,4,4,4,6,6,4,4,6,6,5,5,4,4,3,2,3,2,3,2,3,1,77,}},
	{BLACK_4, {1,2,2,2,2,2,2,2,2,2,4,4,2,2,4,4,3,3,2,2,3,3,2,2,3,3,3,3,5,5,4,4,6,6,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,5,5,4,4,4,4,6,6,4,4,6,6,5,5,4,4,2,3,2,3,2,3,2,1,77,}},
	{BLUE_4, {1,2,2,2,2,2,2,2,2,2,4,4,2,2,4,4,3,3,2,2,3,3,2,2,3,3,3,3,6,6,4,4,5,5,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,4,4,5,5,6,6,4,4,6,6,4,4,4,4,5,5,3,2,3,2,3,2,3,1,77,}},
	{ORANGE_4, {1,2,2,2,2,2,2,2,2,2,4,4,2,2,4,4,3,3,2,2,3,3,2,2,6,6,4,4,3,3,3,4,5,5,4,4,6,6,4,4,5,5,5,5,6,6,6,6,5,5,4,4,6,6,4,4,4,4,5,5,6,6,4,4,6,6,4,4,4,4,5,5,2,3,2,3,2,3,2,1,77,}},
};
static std::vector<uint8_t> CHOREOGRAPHY = choreography_map[MINICUBE.modell_name];
