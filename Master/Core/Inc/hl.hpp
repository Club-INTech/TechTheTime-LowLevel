#pragma once

#include <rpc/controller.hpp>
#include <rpc/def.hpp>

void HL_Start_Frame(rpc::Frame_Type);
void HL_Write_Byte(upd::byte_t);

template<auto &Ftor, typename... Args>
void HL_Call_Remote(Args &&...args) {
	auto key = rpc::controller::keyring.get<Ftor>();
	HL_Start_Frame(rpc::Frame_Type::REQUEST);
	key(std::forward<Args>(args)...) >> HL_Write_Byte;
}

