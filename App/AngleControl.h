/*!
 * @file       AngleControl.h
 * @brief      角度控制函数
 * @author     
 * @version    B车
 * @date       
 */


#ifndef __ANGLECONTROL_H__
#define __ANGLECONTROL_H__

/**************************  声明函数体  **************************************/
void fuzzy_mem_cal(void);
void fuzzy_query(void);
void fuzzy_solve(void);
void steercontrol(void);


/****************************  宏定义  ****************************************/
#define rank (5)  //分级的精确度

#endif  //__ANGLECONTROL_H__
