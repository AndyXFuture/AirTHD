#include "STC15W4K32S4.h"
#include <intrins.h>
#include "delay.h"
//延时子函数区
void delay_ms(uint j)
{
    uint a1,aa1;
		for(a1=j;a1>0;a1--)
	     for(aa1=1200;aa1>0;aa1--) ;
	
}

