#pragma once
#ifndef OutType_H

#define OutType_H

#include<iostream>
#include<stdio.h>

template<typename OutType, typename InType, size_t Cut_Count>
inline OutType BitCut(InType ul, int startIndex)
{
	return static_cast<OutType>(((ul >> startIndex) & (~(~0 << Cut_Count))));
}
#endif // !OutType_H
