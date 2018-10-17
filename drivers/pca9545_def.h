#ifndef PCA9545_DEF_H_
#define PCA9545_DEF_H_

/* define the following for I2C debugging */
//#define PCA9545CDEBUG 1

/* bit positions */

/* CTRL_REG1 - read or write, interrupt status, channel select */
#define PCA9545_INT3                7
#define PCA9545_INT2                6
#define PCA9545_INT1                5
#define PCA9545_INT0                4
#define PCA9545_B3                  3
#define PCA9545_B2                  2
#define PCA9545_B1                  1
#define PCA9545_B0                  0

#endif  // PCA9545_DEF_H_
