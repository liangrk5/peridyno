/**
 * Copyright 2021 Xiaowei He
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "InputModule.h"

#include <queue>

namespace dyno
{
	class MouseInputModule : public InputModule
	{
	public:
		MouseInputModule();
		~MouseInputModule() override;

		void enqueueEvent(PMouseEvent event);

		DEF_VAR(bool, CacheEvent, false, "If set false, all previous queued events will be dropped");

	protected:
		virtual void onEvent(PMouseEvent event) {};

		void updateImpl() final;

		bool requireUpdate() override;

		std::queue<PMouseEvent> mEventQueue;
	};
}
