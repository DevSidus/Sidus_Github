
/* ___   ___   ___        __   ___
|   | | __  |___ |\  | |  \ |   |
|___| |   | |    | \ | |   ||___|
|   | |___| |___ |  \| |__/ |   | version: 1.0
Scheduler library for Arduino
Copyright (c) 2013-2016, Giovanni Blu Mitolo
gioscarab@gmail.com - www.gioblu.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
-  Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
-  All advertising materials mentioning features or use of this software
must display the following acknowledgement:
"Timed by Agenda, developed by Giovanni Blu Mitolo"
-  Neither the name of Agenda nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

This software is provided by the copyright holders and contributors "as is"
and any express or implied warranties, including, but not limited to, the
implied warranties of merchantability and fitness for a particular purpose
are disclaimed. In no event shall the copyright holder or contributors be
liable for any direct, indirect, incidental, special, exemplary, or
consequential damages (including, but not limited to, procurement of
substitute goods or services; loss of use, data, or profits; or business
interruption) however caused and on any theory of liability, whether in
contract, strict liability, or tort (including negligence or otherwise)
arising in any way out of the use of this software, even if advised of
the possibility of such damage. */

#include "Local_Agenda.h"

struct tasks_struct {
	void(*execution)(void);
	unsigned long timing;
	unsigned long registration;
	boolean active;
	boolean once;
	boolean empty;
};

tasks_struct _tasks[MAX_TASKS];

Agenda::Agenda() {
	for (int i = 0; i < MAX_TASKS; i++)
		_tasks[i].empty = true;
}

int Agenda::insert(void(*task)(void), unsigned long timing, boolean once) {
	for (byte i = 0; i < MAX_TASKS; i++)
		if (_tasks[i].empty) {
			_tasks[i].active = true;
			_tasks[i].execution = *task;
			_tasks[i].registration = micros();
			_tasks[i].timing = timing;
			_tasks[i].once = once;
			_tasks[i].empty = false;
			return i;
		}
	return -1;
}

void Agenda::activate(int id) {
	_tasks[id].active = true;
}

void Agenda::deactivate(int id) {
	_tasks[id].active = false;
}

void Agenda::remove(int id) {
	_tasks[id].active = NULL;
	_tasks[id].empty = true;
}

void Agenda::update() {
	// Save time once to reduce execution time
	unsigned long time = micros();
	for (byte i = 0; i < MAX_TASKS; i++)
		if (!_tasks[i].empty && _tasks[i].active)
			if (time - _tasks[i].registration > _tasks[i].timing) {
				_tasks[i].registration = time;
				_tasks[i].execution();
				if (_tasks[i].once)
					this->remove(i);
				time = micros();
				/* Bring time variable back coherent after
				task execution */
			}
}

void Agenda::delay(unsigned long delay) {
	unsigned long time = millis();
	while ((unsigned long)(delay + time) > millis()) {
		this->update();
		//yield(); Arduino ESP-32 library does not support yield() function
	}
}

void Agenda::delay_microseconds(unsigned long delay) {
	unsigned long time = micros();
	while ((unsigned long)(delay + time) > micros()) {
		this->update();
		//yield(); Arduino ESP - 32 library does not support yield() function
	}
}
