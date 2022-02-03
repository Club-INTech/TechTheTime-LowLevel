#pragma once

#include <rpc/controller.hpp>
#include <rpc/def.hpp>

extern "C" {

#include "hl.h"

} // extern "C"

template<auto &Ftor, typename... Args>
void HL_Call_Remote(Args &&...args) {
	for (auto byte : rpc::header) HL_Write_Byte(byte);

	auto key = rpc::controller::keyring.get<Ftor>();
	key(std::forward<Args>(args)...) >> HL_Write_Byte;
}

