#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3393701557564291141) {
   out_3393701557564291141[0] = delta_x[0] + nom_x[0];
   out_3393701557564291141[1] = delta_x[1] + nom_x[1];
   out_3393701557564291141[2] = delta_x[2] + nom_x[2];
   out_3393701557564291141[3] = delta_x[3] + nom_x[3];
   out_3393701557564291141[4] = delta_x[4] + nom_x[4];
   out_3393701557564291141[5] = delta_x[5] + nom_x[5];
   out_3393701557564291141[6] = delta_x[6] + nom_x[6];
   out_3393701557564291141[7] = delta_x[7] + nom_x[7];
   out_3393701557564291141[8] = delta_x[8] + nom_x[8];
   out_3393701557564291141[9] = delta_x[9] + nom_x[9];
   out_3393701557564291141[10] = delta_x[10] + nom_x[10];
   out_3393701557564291141[11] = delta_x[11] + nom_x[11];
   out_3393701557564291141[12] = delta_x[12] + nom_x[12];
   out_3393701557564291141[13] = delta_x[13] + nom_x[13];
   out_3393701557564291141[14] = delta_x[14] + nom_x[14];
   out_3393701557564291141[15] = delta_x[15] + nom_x[15];
   out_3393701557564291141[16] = delta_x[16] + nom_x[16];
   out_3393701557564291141[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6525324592803266903) {
   out_6525324592803266903[0] = -nom_x[0] + true_x[0];
   out_6525324592803266903[1] = -nom_x[1] + true_x[1];
   out_6525324592803266903[2] = -nom_x[2] + true_x[2];
   out_6525324592803266903[3] = -nom_x[3] + true_x[3];
   out_6525324592803266903[4] = -nom_x[4] + true_x[4];
   out_6525324592803266903[5] = -nom_x[5] + true_x[5];
   out_6525324592803266903[6] = -nom_x[6] + true_x[6];
   out_6525324592803266903[7] = -nom_x[7] + true_x[7];
   out_6525324592803266903[8] = -nom_x[8] + true_x[8];
   out_6525324592803266903[9] = -nom_x[9] + true_x[9];
   out_6525324592803266903[10] = -nom_x[10] + true_x[10];
   out_6525324592803266903[11] = -nom_x[11] + true_x[11];
   out_6525324592803266903[12] = -nom_x[12] + true_x[12];
   out_6525324592803266903[13] = -nom_x[13] + true_x[13];
   out_6525324592803266903[14] = -nom_x[14] + true_x[14];
   out_6525324592803266903[15] = -nom_x[15] + true_x[15];
   out_6525324592803266903[16] = -nom_x[16] + true_x[16];
   out_6525324592803266903[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_9151286745456408536) {
   out_9151286745456408536[0] = 1.0;
   out_9151286745456408536[1] = 0.0;
   out_9151286745456408536[2] = 0.0;
   out_9151286745456408536[3] = 0.0;
   out_9151286745456408536[4] = 0.0;
   out_9151286745456408536[5] = 0.0;
   out_9151286745456408536[6] = 0.0;
   out_9151286745456408536[7] = 0.0;
   out_9151286745456408536[8] = 0.0;
   out_9151286745456408536[9] = 0.0;
   out_9151286745456408536[10] = 0.0;
   out_9151286745456408536[11] = 0.0;
   out_9151286745456408536[12] = 0.0;
   out_9151286745456408536[13] = 0.0;
   out_9151286745456408536[14] = 0.0;
   out_9151286745456408536[15] = 0.0;
   out_9151286745456408536[16] = 0.0;
   out_9151286745456408536[17] = 0.0;
   out_9151286745456408536[18] = 0.0;
   out_9151286745456408536[19] = 1.0;
   out_9151286745456408536[20] = 0.0;
   out_9151286745456408536[21] = 0.0;
   out_9151286745456408536[22] = 0.0;
   out_9151286745456408536[23] = 0.0;
   out_9151286745456408536[24] = 0.0;
   out_9151286745456408536[25] = 0.0;
   out_9151286745456408536[26] = 0.0;
   out_9151286745456408536[27] = 0.0;
   out_9151286745456408536[28] = 0.0;
   out_9151286745456408536[29] = 0.0;
   out_9151286745456408536[30] = 0.0;
   out_9151286745456408536[31] = 0.0;
   out_9151286745456408536[32] = 0.0;
   out_9151286745456408536[33] = 0.0;
   out_9151286745456408536[34] = 0.0;
   out_9151286745456408536[35] = 0.0;
   out_9151286745456408536[36] = 0.0;
   out_9151286745456408536[37] = 0.0;
   out_9151286745456408536[38] = 1.0;
   out_9151286745456408536[39] = 0.0;
   out_9151286745456408536[40] = 0.0;
   out_9151286745456408536[41] = 0.0;
   out_9151286745456408536[42] = 0.0;
   out_9151286745456408536[43] = 0.0;
   out_9151286745456408536[44] = 0.0;
   out_9151286745456408536[45] = 0.0;
   out_9151286745456408536[46] = 0.0;
   out_9151286745456408536[47] = 0.0;
   out_9151286745456408536[48] = 0.0;
   out_9151286745456408536[49] = 0.0;
   out_9151286745456408536[50] = 0.0;
   out_9151286745456408536[51] = 0.0;
   out_9151286745456408536[52] = 0.0;
   out_9151286745456408536[53] = 0.0;
   out_9151286745456408536[54] = 0.0;
   out_9151286745456408536[55] = 0.0;
   out_9151286745456408536[56] = 0.0;
   out_9151286745456408536[57] = 1.0;
   out_9151286745456408536[58] = 0.0;
   out_9151286745456408536[59] = 0.0;
   out_9151286745456408536[60] = 0.0;
   out_9151286745456408536[61] = 0.0;
   out_9151286745456408536[62] = 0.0;
   out_9151286745456408536[63] = 0.0;
   out_9151286745456408536[64] = 0.0;
   out_9151286745456408536[65] = 0.0;
   out_9151286745456408536[66] = 0.0;
   out_9151286745456408536[67] = 0.0;
   out_9151286745456408536[68] = 0.0;
   out_9151286745456408536[69] = 0.0;
   out_9151286745456408536[70] = 0.0;
   out_9151286745456408536[71] = 0.0;
   out_9151286745456408536[72] = 0.0;
   out_9151286745456408536[73] = 0.0;
   out_9151286745456408536[74] = 0.0;
   out_9151286745456408536[75] = 0.0;
   out_9151286745456408536[76] = 1.0;
   out_9151286745456408536[77] = 0.0;
   out_9151286745456408536[78] = 0.0;
   out_9151286745456408536[79] = 0.0;
   out_9151286745456408536[80] = 0.0;
   out_9151286745456408536[81] = 0.0;
   out_9151286745456408536[82] = 0.0;
   out_9151286745456408536[83] = 0.0;
   out_9151286745456408536[84] = 0.0;
   out_9151286745456408536[85] = 0.0;
   out_9151286745456408536[86] = 0.0;
   out_9151286745456408536[87] = 0.0;
   out_9151286745456408536[88] = 0.0;
   out_9151286745456408536[89] = 0.0;
   out_9151286745456408536[90] = 0.0;
   out_9151286745456408536[91] = 0.0;
   out_9151286745456408536[92] = 0.0;
   out_9151286745456408536[93] = 0.0;
   out_9151286745456408536[94] = 0.0;
   out_9151286745456408536[95] = 1.0;
   out_9151286745456408536[96] = 0.0;
   out_9151286745456408536[97] = 0.0;
   out_9151286745456408536[98] = 0.0;
   out_9151286745456408536[99] = 0.0;
   out_9151286745456408536[100] = 0.0;
   out_9151286745456408536[101] = 0.0;
   out_9151286745456408536[102] = 0.0;
   out_9151286745456408536[103] = 0.0;
   out_9151286745456408536[104] = 0.0;
   out_9151286745456408536[105] = 0.0;
   out_9151286745456408536[106] = 0.0;
   out_9151286745456408536[107] = 0.0;
   out_9151286745456408536[108] = 0.0;
   out_9151286745456408536[109] = 0.0;
   out_9151286745456408536[110] = 0.0;
   out_9151286745456408536[111] = 0.0;
   out_9151286745456408536[112] = 0.0;
   out_9151286745456408536[113] = 0.0;
   out_9151286745456408536[114] = 1.0;
   out_9151286745456408536[115] = 0.0;
   out_9151286745456408536[116] = 0.0;
   out_9151286745456408536[117] = 0.0;
   out_9151286745456408536[118] = 0.0;
   out_9151286745456408536[119] = 0.0;
   out_9151286745456408536[120] = 0.0;
   out_9151286745456408536[121] = 0.0;
   out_9151286745456408536[122] = 0.0;
   out_9151286745456408536[123] = 0.0;
   out_9151286745456408536[124] = 0.0;
   out_9151286745456408536[125] = 0.0;
   out_9151286745456408536[126] = 0.0;
   out_9151286745456408536[127] = 0.0;
   out_9151286745456408536[128] = 0.0;
   out_9151286745456408536[129] = 0.0;
   out_9151286745456408536[130] = 0.0;
   out_9151286745456408536[131] = 0.0;
   out_9151286745456408536[132] = 0.0;
   out_9151286745456408536[133] = 1.0;
   out_9151286745456408536[134] = 0.0;
   out_9151286745456408536[135] = 0.0;
   out_9151286745456408536[136] = 0.0;
   out_9151286745456408536[137] = 0.0;
   out_9151286745456408536[138] = 0.0;
   out_9151286745456408536[139] = 0.0;
   out_9151286745456408536[140] = 0.0;
   out_9151286745456408536[141] = 0.0;
   out_9151286745456408536[142] = 0.0;
   out_9151286745456408536[143] = 0.0;
   out_9151286745456408536[144] = 0.0;
   out_9151286745456408536[145] = 0.0;
   out_9151286745456408536[146] = 0.0;
   out_9151286745456408536[147] = 0.0;
   out_9151286745456408536[148] = 0.0;
   out_9151286745456408536[149] = 0.0;
   out_9151286745456408536[150] = 0.0;
   out_9151286745456408536[151] = 0.0;
   out_9151286745456408536[152] = 1.0;
   out_9151286745456408536[153] = 0.0;
   out_9151286745456408536[154] = 0.0;
   out_9151286745456408536[155] = 0.0;
   out_9151286745456408536[156] = 0.0;
   out_9151286745456408536[157] = 0.0;
   out_9151286745456408536[158] = 0.0;
   out_9151286745456408536[159] = 0.0;
   out_9151286745456408536[160] = 0.0;
   out_9151286745456408536[161] = 0.0;
   out_9151286745456408536[162] = 0.0;
   out_9151286745456408536[163] = 0.0;
   out_9151286745456408536[164] = 0.0;
   out_9151286745456408536[165] = 0.0;
   out_9151286745456408536[166] = 0.0;
   out_9151286745456408536[167] = 0.0;
   out_9151286745456408536[168] = 0.0;
   out_9151286745456408536[169] = 0.0;
   out_9151286745456408536[170] = 0.0;
   out_9151286745456408536[171] = 1.0;
   out_9151286745456408536[172] = 0.0;
   out_9151286745456408536[173] = 0.0;
   out_9151286745456408536[174] = 0.0;
   out_9151286745456408536[175] = 0.0;
   out_9151286745456408536[176] = 0.0;
   out_9151286745456408536[177] = 0.0;
   out_9151286745456408536[178] = 0.0;
   out_9151286745456408536[179] = 0.0;
   out_9151286745456408536[180] = 0.0;
   out_9151286745456408536[181] = 0.0;
   out_9151286745456408536[182] = 0.0;
   out_9151286745456408536[183] = 0.0;
   out_9151286745456408536[184] = 0.0;
   out_9151286745456408536[185] = 0.0;
   out_9151286745456408536[186] = 0.0;
   out_9151286745456408536[187] = 0.0;
   out_9151286745456408536[188] = 0.0;
   out_9151286745456408536[189] = 0.0;
   out_9151286745456408536[190] = 1.0;
   out_9151286745456408536[191] = 0.0;
   out_9151286745456408536[192] = 0.0;
   out_9151286745456408536[193] = 0.0;
   out_9151286745456408536[194] = 0.0;
   out_9151286745456408536[195] = 0.0;
   out_9151286745456408536[196] = 0.0;
   out_9151286745456408536[197] = 0.0;
   out_9151286745456408536[198] = 0.0;
   out_9151286745456408536[199] = 0.0;
   out_9151286745456408536[200] = 0.0;
   out_9151286745456408536[201] = 0.0;
   out_9151286745456408536[202] = 0.0;
   out_9151286745456408536[203] = 0.0;
   out_9151286745456408536[204] = 0.0;
   out_9151286745456408536[205] = 0.0;
   out_9151286745456408536[206] = 0.0;
   out_9151286745456408536[207] = 0.0;
   out_9151286745456408536[208] = 0.0;
   out_9151286745456408536[209] = 1.0;
   out_9151286745456408536[210] = 0.0;
   out_9151286745456408536[211] = 0.0;
   out_9151286745456408536[212] = 0.0;
   out_9151286745456408536[213] = 0.0;
   out_9151286745456408536[214] = 0.0;
   out_9151286745456408536[215] = 0.0;
   out_9151286745456408536[216] = 0.0;
   out_9151286745456408536[217] = 0.0;
   out_9151286745456408536[218] = 0.0;
   out_9151286745456408536[219] = 0.0;
   out_9151286745456408536[220] = 0.0;
   out_9151286745456408536[221] = 0.0;
   out_9151286745456408536[222] = 0.0;
   out_9151286745456408536[223] = 0.0;
   out_9151286745456408536[224] = 0.0;
   out_9151286745456408536[225] = 0.0;
   out_9151286745456408536[226] = 0.0;
   out_9151286745456408536[227] = 0.0;
   out_9151286745456408536[228] = 1.0;
   out_9151286745456408536[229] = 0.0;
   out_9151286745456408536[230] = 0.0;
   out_9151286745456408536[231] = 0.0;
   out_9151286745456408536[232] = 0.0;
   out_9151286745456408536[233] = 0.0;
   out_9151286745456408536[234] = 0.0;
   out_9151286745456408536[235] = 0.0;
   out_9151286745456408536[236] = 0.0;
   out_9151286745456408536[237] = 0.0;
   out_9151286745456408536[238] = 0.0;
   out_9151286745456408536[239] = 0.0;
   out_9151286745456408536[240] = 0.0;
   out_9151286745456408536[241] = 0.0;
   out_9151286745456408536[242] = 0.0;
   out_9151286745456408536[243] = 0.0;
   out_9151286745456408536[244] = 0.0;
   out_9151286745456408536[245] = 0.0;
   out_9151286745456408536[246] = 0.0;
   out_9151286745456408536[247] = 1.0;
   out_9151286745456408536[248] = 0.0;
   out_9151286745456408536[249] = 0.0;
   out_9151286745456408536[250] = 0.0;
   out_9151286745456408536[251] = 0.0;
   out_9151286745456408536[252] = 0.0;
   out_9151286745456408536[253] = 0.0;
   out_9151286745456408536[254] = 0.0;
   out_9151286745456408536[255] = 0.0;
   out_9151286745456408536[256] = 0.0;
   out_9151286745456408536[257] = 0.0;
   out_9151286745456408536[258] = 0.0;
   out_9151286745456408536[259] = 0.0;
   out_9151286745456408536[260] = 0.0;
   out_9151286745456408536[261] = 0.0;
   out_9151286745456408536[262] = 0.0;
   out_9151286745456408536[263] = 0.0;
   out_9151286745456408536[264] = 0.0;
   out_9151286745456408536[265] = 0.0;
   out_9151286745456408536[266] = 1.0;
   out_9151286745456408536[267] = 0.0;
   out_9151286745456408536[268] = 0.0;
   out_9151286745456408536[269] = 0.0;
   out_9151286745456408536[270] = 0.0;
   out_9151286745456408536[271] = 0.0;
   out_9151286745456408536[272] = 0.0;
   out_9151286745456408536[273] = 0.0;
   out_9151286745456408536[274] = 0.0;
   out_9151286745456408536[275] = 0.0;
   out_9151286745456408536[276] = 0.0;
   out_9151286745456408536[277] = 0.0;
   out_9151286745456408536[278] = 0.0;
   out_9151286745456408536[279] = 0.0;
   out_9151286745456408536[280] = 0.0;
   out_9151286745456408536[281] = 0.0;
   out_9151286745456408536[282] = 0.0;
   out_9151286745456408536[283] = 0.0;
   out_9151286745456408536[284] = 0.0;
   out_9151286745456408536[285] = 1.0;
   out_9151286745456408536[286] = 0.0;
   out_9151286745456408536[287] = 0.0;
   out_9151286745456408536[288] = 0.0;
   out_9151286745456408536[289] = 0.0;
   out_9151286745456408536[290] = 0.0;
   out_9151286745456408536[291] = 0.0;
   out_9151286745456408536[292] = 0.0;
   out_9151286745456408536[293] = 0.0;
   out_9151286745456408536[294] = 0.0;
   out_9151286745456408536[295] = 0.0;
   out_9151286745456408536[296] = 0.0;
   out_9151286745456408536[297] = 0.0;
   out_9151286745456408536[298] = 0.0;
   out_9151286745456408536[299] = 0.0;
   out_9151286745456408536[300] = 0.0;
   out_9151286745456408536[301] = 0.0;
   out_9151286745456408536[302] = 0.0;
   out_9151286745456408536[303] = 0.0;
   out_9151286745456408536[304] = 1.0;
   out_9151286745456408536[305] = 0.0;
   out_9151286745456408536[306] = 0.0;
   out_9151286745456408536[307] = 0.0;
   out_9151286745456408536[308] = 0.0;
   out_9151286745456408536[309] = 0.0;
   out_9151286745456408536[310] = 0.0;
   out_9151286745456408536[311] = 0.0;
   out_9151286745456408536[312] = 0.0;
   out_9151286745456408536[313] = 0.0;
   out_9151286745456408536[314] = 0.0;
   out_9151286745456408536[315] = 0.0;
   out_9151286745456408536[316] = 0.0;
   out_9151286745456408536[317] = 0.0;
   out_9151286745456408536[318] = 0.0;
   out_9151286745456408536[319] = 0.0;
   out_9151286745456408536[320] = 0.0;
   out_9151286745456408536[321] = 0.0;
   out_9151286745456408536[322] = 0.0;
   out_9151286745456408536[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8343874082406999130) {
   out_8343874082406999130[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8343874082406999130[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8343874082406999130[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8343874082406999130[3] = dt*state[12] + state[3];
   out_8343874082406999130[4] = dt*state[13] + state[4];
   out_8343874082406999130[5] = dt*state[14] + state[5];
   out_8343874082406999130[6] = state[6];
   out_8343874082406999130[7] = state[7];
   out_8343874082406999130[8] = state[8];
   out_8343874082406999130[9] = state[9];
   out_8343874082406999130[10] = state[10];
   out_8343874082406999130[11] = state[11];
   out_8343874082406999130[12] = state[12];
   out_8343874082406999130[13] = state[13];
   out_8343874082406999130[14] = state[14];
   out_8343874082406999130[15] = state[15];
   out_8343874082406999130[16] = state[16];
   out_8343874082406999130[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3953690814815203097) {
   out_3953690814815203097[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3953690814815203097[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3953690814815203097[2] = 0;
   out_3953690814815203097[3] = 0;
   out_3953690814815203097[4] = 0;
   out_3953690814815203097[5] = 0;
   out_3953690814815203097[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3953690814815203097[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3953690814815203097[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3953690814815203097[9] = 0;
   out_3953690814815203097[10] = 0;
   out_3953690814815203097[11] = 0;
   out_3953690814815203097[12] = 0;
   out_3953690814815203097[13] = 0;
   out_3953690814815203097[14] = 0;
   out_3953690814815203097[15] = 0;
   out_3953690814815203097[16] = 0;
   out_3953690814815203097[17] = 0;
   out_3953690814815203097[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3953690814815203097[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3953690814815203097[20] = 0;
   out_3953690814815203097[21] = 0;
   out_3953690814815203097[22] = 0;
   out_3953690814815203097[23] = 0;
   out_3953690814815203097[24] = 0;
   out_3953690814815203097[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3953690814815203097[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3953690814815203097[27] = 0;
   out_3953690814815203097[28] = 0;
   out_3953690814815203097[29] = 0;
   out_3953690814815203097[30] = 0;
   out_3953690814815203097[31] = 0;
   out_3953690814815203097[32] = 0;
   out_3953690814815203097[33] = 0;
   out_3953690814815203097[34] = 0;
   out_3953690814815203097[35] = 0;
   out_3953690814815203097[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3953690814815203097[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3953690814815203097[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3953690814815203097[39] = 0;
   out_3953690814815203097[40] = 0;
   out_3953690814815203097[41] = 0;
   out_3953690814815203097[42] = 0;
   out_3953690814815203097[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3953690814815203097[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3953690814815203097[45] = 0;
   out_3953690814815203097[46] = 0;
   out_3953690814815203097[47] = 0;
   out_3953690814815203097[48] = 0;
   out_3953690814815203097[49] = 0;
   out_3953690814815203097[50] = 0;
   out_3953690814815203097[51] = 0;
   out_3953690814815203097[52] = 0;
   out_3953690814815203097[53] = 0;
   out_3953690814815203097[54] = 0;
   out_3953690814815203097[55] = 0;
   out_3953690814815203097[56] = 0;
   out_3953690814815203097[57] = 1;
   out_3953690814815203097[58] = 0;
   out_3953690814815203097[59] = 0;
   out_3953690814815203097[60] = 0;
   out_3953690814815203097[61] = 0;
   out_3953690814815203097[62] = 0;
   out_3953690814815203097[63] = 0;
   out_3953690814815203097[64] = 0;
   out_3953690814815203097[65] = 0;
   out_3953690814815203097[66] = dt;
   out_3953690814815203097[67] = 0;
   out_3953690814815203097[68] = 0;
   out_3953690814815203097[69] = 0;
   out_3953690814815203097[70] = 0;
   out_3953690814815203097[71] = 0;
   out_3953690814815203097[72] = 0;
   out_3953690814815203097[73] = 0;
   out_3953690814815203097[74] = 0;
   out_3953690814815203097[75] = 0;
   out_3953690814815203097[76] = 1;
   out_3953690814815203097[77] = 0;
   out_3953690814815203097[78] = 0;
   out_3953690814815203097[79] = 0;
   out_3953690814815203097[80] = 0;
   out_3953690814815203097[81] = 0;
   out_3953690814815203097[82] = 0;
   out_3953690814815203097[83] = 0;
   out_3953690814815203097[84] = 0;
   out_3953690814815203097[85] = dt;
   out_3953690814815203097[86] = 0;
   out_3953690814815203097[87] = 0;
   out_3953690814815203097[88] = 0;
   out_3953690814815203097[89] = 0;
   out_3953690814815203097[90] = 0;
   out_3953690814815203097[91] = 0;
   out_3953690814815203097[92] = 0;
   out_3953690814815203097[93] = 0;
   out_3953690814815203097[94] = 0;
   out_3953690814815203097[95] = 1;
   out_3953690814815203097[96] = 0;
   out_3953690814815203097[97] = 0;
   out_3953690814815203097[98] = 0;
   out_3953690814815203097[99] = 0;
   out_3953690814815203097[100] = 0;
   out_3953690814815203097[101] = 0;
   out_3953690814815203097[102] = 0;
   out_3953690814815203097[103] = 0;
   out_3953690814815203097[104] = dt;
   out_3953690814815203097[105] = 0;
   out_3953690814815203097[106] = 0;
   out_3953690814815203097[107] = 0;
   out_3953690814815203097[108] = 0;
   out_3953690814815203097[109] = 0;
   out_3953690814815203097[110] = 0;
   out_3953690814815203097[111] = 0;
   out_3953690814815203097[112] = 0;
   out_3953690814815203097[113] = 0;
   out_3953690814815203097[114] = 1;
   out_3953690814815203097[115] = 0;
   out_3953690814815203097[116] = 0;
   out_3953690814815203097[117] = 0;
   out_3953690814815203097[118] = 0;
   out_3953690814815203097[119] = 0;
   out_3953690814815203097[120] = 0;
   out_3953690814815203097[121] = 0;
   out_3953690814815203097[122] = 0;
   out_3953690814815203097[123] = 0;
   out_3953690814815203097[124] = 0;
   out_3953690814815203097[125] = 0;
   out_3953690814815203097[126] = 0;
   out_3953690814815203097[127] = 0;
   out_3953690814815203097[128] = 0;
   out_3953690814815203097[129] = 0;
   out_3953690814815203097[130] = 0;
   out_3953690814815203097[131] = 0;
   out_3953690814815203097[132] = 0;
   out_3953690814815203097[133] = 1;
   out_3953690814815203097[134] = 0;
   out_3953690814815203097[135] = 0;
   out_3953690814815203097[136] = 0;
   out_3953690814815203097[137] = 0;
   out_3953690814815203097[138] = 0;
   out_3953690814815203097[139] = 0;
   out_3953690814815203097[140] = 0;
   out_3953690814815203097[141] = 0;
   out_3953690814815203097[142] = 0;
   out_3953690814815203097[143] = 0;
   out_3953690814815203097[144] = 0;
   out_3953690814815203097[145] = 0;
   out_3953690814815203097[146] = 0;
   out_3953690814815203097[147] = 0;
   out_3953690814815203097[148] = 0;
   out_3953690814815203097[149] = 0;
   out_3953690814815203097[150] = 0;
   out_3953690814815203097[151] = 0;
   out_3953690814815203097[152] = 1;
   out_3953690814815203097[153] = 0;
   out_3953690814815203097[154] = 0;
   out_3953690814815203097[155] = 0;
   out_3953690814815203097[156] = 0;
   out_3953690814815203097[157] = 0;
   out_3953690814815203097[158] = 0;
   out_3953690814815203097[159] = 0;
   out_3953690814815203097[160] = 0;
   out_3953690814815203097[161] = 0;
   out_3953690814815203097[162] = 0;
   out_3953690814815203097[163] = 0;
   out_3953690814815203097[164] = 0;
   out_3953690814815203097[165] = 0;
   out_3953690814815203097[166] = 0;
   out_3953690814815203097[167] = 0;
   out_3953690814815203097[168] = 0;
   out_3953690814815203097[169] = 0;
   out_3953690814815203097[170] = 0;
   out_3953690814815203097[171] = 1;
   out_3953690814815203097[172] = 0;
   out_3953690814815203097[173] = 0;
   out_3953690814815203097[174] = 0;
   out_3953690814815203097[175] = 0;
   out_3953690814815203097[176] = 0;
   out_3953690814815203097[177] = 0;
   out_3953690814815203097[178] = 0;
   out_3953690814815203097[179] = 0;
   out_3953690814815203097[180] = 0;
   out_3953690814815203097[181] = 0;
   out_3953690814815203097[182] = 0;
   out_3953690814815203097[183] = 0;
   out_3953690814815203097[184] = 0;
   out_3953690814815203097[185] = 0;
   out_3953690814815203097[186] = 0;
   out_3953690814815203097[187] = 0;
   out_3953690814815203097[188] = 0;
   out_3953690814815203097[189] = 0;
   out_3953690814815203097[190] = 1;
   out_3953690814815203097[191] = 0;
   out_3953690814815203097[192] = 0;
   out_3953690814815203097[193] = 0;
   out_3953690814815203097[194] = 0;
   out_3953690814815203097[195] = 0;
   out_3953690814815203097[196] = 0;
   out_3953690814815203097[197] = 0;
   out_3953690814815203097[198] = 0;
   out_3953690814815203097[199] = 0;
   out_3953690814815203097[200] = 0;
   out_3953690814815203097[201] = 0;
   out_3953690814815203097[202] = 0;
   out_3953690814815203097[203] = 0;
   out_3953690814815203097[204] = 0;
   out_3953690814815203097[205] = 0;
   out_3953690814815203097[206] = 0;
   out_3953690814815203097[207] = 0;
   out_3953690814815203097[208] = 0;
   out_3953690814815203097[209] = 1;
   out_3953690814815203097[210] = 0;
   out_3953690814815203097[211] = 0;
   out_3953690814815203097[212] = 0;
   out_3953690814815203097[213] = 0;
   out_3953690814815203097[214] = 0;
   out_3953690814815203097[215] = 0;
   out_3953690814815203097[216] = 0;
   out_3953690814815203097[217] = 0;
   out_3953690814815203097[218] = 0;
   out_3953690814815203097[219] = 0;
   out_3953690814815203097[220] = 0;
   out_3953690814815203097[221] = 0;
   out_3953690814815203097[222] = 0;
   out_3953690814815203097[223] = 0;
   out_3953690814815203097[224] = 0;
   out_3953690814815203097[225] = 0;
   out_3953690814815203097[226] = 0;
   out_3953690814815203097[227] = 0;
   out_3953690814815203097[228] = 1;
   out_3953690814815203097[229] = 0;
   out_3953690814815203097[230] = 0;
   out_3953690814815203097[231] = 0;
   out_3953690814815203097[232] = 0;
   out_3953690814815203097[233] = 0;
   out_3953690814815203097[234] = 0;
   out_3953690814815203097[235] = 0;
   out_3953690814815203097[236] = 0;
   out_3953690814815203097[237] = 0;
   out_3953690814815203097[238] = 0;
   out_3953690814815203097[239] = 0;
   out_3953690814815203097[240] = 0;
   out_3953690814815203097[241] = 0;
   out_3953690814815203097[242] = 0;
   out_3953690814815203097[243] = 0;
   out_3953690814815203097[244] = 0;
   out_3953690814815203097[245] = 0;
   out_3953690814815203097[246] = 0;
   out_3953690814815203097[247] = 1;
   out_3953690814815203097[248] = 0;
   out_3953690814815203097[249] = 0;
   out_3953690814815203097[250] = 0;
   out_3953690814815203097[251] = 0;
   out_3953690814815203097[252] = 0;
   out_3953690814815203097[253] = 0;
   out_3953690814815203097[254] = 0;
   out_3953690814815203097[255] = 0;
   out_3953690814815203097[256] = 0;
   out_3953690814815203097[257] = 0;
   out_3953690814815203097[258] = 0;
   out_3953690814815203097[259] = 0;
   out_3953690814815203097[260] = 0;
   out_3953690814815203097[261] = 0;
   out_3953690814815203097[262] = 0;
   out_3953690814815203097[263] = 0;
   out_3953690814815203097[264] = 0;
   out_3953690814815203097[265] = 0;
   out_3953690814815203097[266] = 1;
   out_3953690814815203097[267] = 0;
   out_3953690814815203097[268] = 0;
   out_3953690814815203097[269] = 0;
   out_3953690814815203097[270] = 0;
   out_3953690814815203097[271] = 0;
   out_3953690814815203097[272] = 0;
   out_3953690814815203097[273] = 0;
   out_3953690814815203097[274] = 0;
   out_3953690814815203097[275] = 0;
   out_3953690814815203097[276] = 0;
   out_3953690814815203097[277] = 0;
   out_3953690814815203097[278] = 0;
   out_3953690814815203097[279] = 0;
   out_3953690814815203097[280] = 0;
   out_3953690814815203097[281] = 0;
   out_3953690814815203097[282] = 0;
   out_3953690814815203097[283] = 0;
   out_3953690814815203097[284] = 0;
   out_3953690814815203097[285] = 1;
   out_3953690814815203097[286] = 0;
   out_3953690814815203097[287] = 0;
   out_3953690814815203097[288] = 0;
   out_3953690814815203097[289] = 0;
   out_3953690814815203097[290] = 0;
   out_3953690814815203097[291] = 0;
   out_3953690814815203097[292] = 0;
   out_3953690814815203097[293] = 0;
   out_3953690814815203097[294] = 0;
   out_3953690814815203097[295] = 0;
   out_3953690814815203097[296] = 0;
   out_3953690814815203097[297] = 0;
   out_3953690814815203097[298] = 0;
   out_3953690814815203097[299] = 0;
   out_3953690814815203097[300] = 0;
   out_3953690814815203097[301] = 0;
   out_3953690814815203097[302] = 0;
   out_3953690814815203097[303] = 0;
   out_3953690814815203097[304] = 1;
   out_3953690814815203097[305] = 0;
   out_3953690814815203097[306] = 0;
   out_3953690814815203097[307] = 0;
   out_3953690814815203097[308] = 0;
   out_3953690814815203097[309] = 0;
   out_3953690814815203097[310] = 0;
   out_3953690814815203097[311] = 0;
   out_3953690814815203097[312] = 0;
   out_3953690814815203097[313] = 0;
   out_3953690814815203097[314] = 0;
   out_3953690814815203097[315] = 0;
   out_3953690814815203097[316] = 0;
   out_3953690814815203097[317] = 0;
   out_3953690814815203097[318] = 0;
   out_3953690814815203097[319] = 0;
   out_3953690814815203097[320] = 0;
   out_3953690814815203097[321] = 0;
   out_3953690814815203097[322] = 0;
   out_3953690814815203097[323] = 1;
}
void h_4(double *state, double *unused, double *out_3795290653571420264) {
   out_3795290653571420264[0] = state[6] + state[9];
   out_3795290653571420264[1] = state[7] + state[10];
   out_3795290653571420264[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3998768360417327801) {
   out_3998768360417327801[0] = 0;
   out_3998768360417327801[1] = 0;
   out_3998768360417327801[2] = 0;
   out_3998768360417327801[3] = 0;
   out_3998768360417327801[4] = 0;
   out_3998768360417327801[5] = 0;
   out_3998768360417327801[6] = 1;
   out_3998768360417327801[7] = 0;
   out_3998768360417327801[8] = 0;
   out_3998768360417327801[9] = 1;
   out_3998768360417327801[10] = 0;
   out_3998768360417327801[11] = 0;
   out_3998768360417327801[12] = 0;
   out_3998768360417327801[13] = 0;
   out_3998768360417327801[14] = 0;
   out_3998768360417327801[15] = 0;
   out_3998768360417327801[16] = 0;
   out_3998768360417327801[17] = 0;
   out_3998768360417327801[18] = 0;
   out_3998768360417327801[19] = 0;
   out_3998768360417327801[20] = 0;
   out_3998768360417327801[21] = 0;
   out_3998768360417327801[22] = 0;
   out_3998768360417327801[23] = 0;
   out_3998768360417327801[24] = 0;
   out_3998768360417327801[25] = 1;
   out_3998768360417327801[26] = 0;
   out_3998768360417327801[27] = 0;
   out_3998768360417327801[28] = 1;
   out_3998768360417327801[29] = 0;
   out_3998768360417327801[30] = 0;
   out_3998768360417327801[31] = 0;
   out_3998768360417327801[32] = 0;
   out_3998768360417327801[33] = 0;
   out_3998768360417327801[34] = 0;
   out_3998768360417327801[35] = 0;
   out_3998768360417327801[36] = 0;
   out_3998768360417327801[37] = 0;
   out_3998768360417327801[38] = 0;
   out_3998768360417327801[39] = 0;
   out_3998768360417327801[40] = 0;
   out_3998768360417327801[41] = 0;
   out_3998768360417327801[42] = 0;
   out_3998768360417327801[43] = 0;
   out_3998768360417327801[44] = 1;
   out_3998768360417327801[45] = 0;
   out_3998768360417327801[46] = 0;
   out_3998768360417327801[47] = 1;
   out_3998768360417327801[48] = 0;
   out_3998768360417327801[49] = 0;
   out_3998768360417327801[50] = 0;
   out_3998768360417327801[51] = 0;
   out_3998768360417327801[52] = 0;
   out_3998768360417327801[53] = 0;
}
void h_10(double *state, double *unused, double *out_2903431963768148397) {
   out_2903431963768148397[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2903431963768148397[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2903431963768148397[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3193504043246302910) {
   out_3193504043246302910[0] = 0;
   out_3193504043246302910[1] = 9.8100000000000005*cos(state[1]);
   out_3193504043246302910[2] = 0;
   out_3193504043246302910[3] = 0;
   out_3193504043246302910[4] = -state[8];
   out_3193504043246302910[5] = state[7];
   out_3193504043246302910[6] = 0;
   out_3193504043246302910[7] = state[5];
   out_3193504043246302910[8] = -state[4];
   out_3193504043246302910[9] = 0;
   out_3193504043246302910[10] = 0;
   out_3193504043246302910[11] = 0;
   out_3193504043246302910[12] = 1;
   out_3193504043246302910[13] = 0;
   out_3193504043246302910[14] = 0;
   out_3193504043246302910[15] = 1;
   out_3193504043246302910[16] = 0;
   out_3193504043246302910[17] = 0;
   out_3193504043246302910[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3193504043246302910[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3193504043246302910[20] = 0;
   out_3193504043246302910[21] = state[8];
   out_3193504043246302910[22] = 0;
   out_3193504043246302910[23] = -state[6];
   out_3193504043246302910[24] = -state[5];
   out_3193504043246302910[25] = 0;
   out_3193504043246302910[26] = state[3];
   out_3193504043246302910[27] = 0;
   out_3193504043246302910[28] = 0;
   out_3193504043246302910[29] = 0;
   out_3193504043246302910[30] = 0;
   out_3193504043246302910[31] = 1;
   out_3193504043246302910[32] = 0;
   out_3193504043246302910[33] = 0;
   out_3193504043246302910[34] = 1;
   out_3193504043246302910[35] = 0;
   out_3193504043246302910[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3193504043246302910[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3193504043246302910[38] = 0;
   out_3193504043246302910[39] = -state[7];
   out_3193504043246302910[40] = state[6];
   out_3193504043246302910[41] = 0;
   out_3193504043246302910[42] = state[4];
   out_3193504043246302910[43] = -state[3];
   out_3193504043246302910[44] = 0;
   out_3193504043246302910[45] = 0;
   out_3193504043246302910[46] = 0;
   out_3193504043246302910[47] = 0;
   out_3193504043246302910[48] = 0;
   out_3193504043246302910[49] = 0;
   out_3193504043246302910[50] = 1;
   out_3193504043246302910[51] = 0;
   out_3193504043246302910[52] = 0;
   out_3193504043246302910[53] = 1;
}
void h_13(double *state, double *unused, double *out_7802322127374541928) {
   out_7802322127374541928[0] = state[3];
   out_7802322127374541928[1] = state[4];
   out_7802322127374541928[2] = state[5];
}
void H_13(double *state, double *unused, double *out_786494535084995000) {
   out_786494535084995000[0] = 0;
   out_786494535084995000[1] = 0;
   out_786494535084995000[2] = 0;
   out_786494535084995000[3] = 1;
   out_786494535084995000[4] = 0;
   out_786494535084995000[5] = 0;
   out_786494535084995000[6] = 0;
   out_786494535084995000[7] = 0;
   out_786494535084995000[8] = 0;
   out_786494535084995000[9] = 0;
   out_786494535084995000[10] = 0;
   out_786494535084995000[11] = 0;
   out_786494535084995000[12] = 0;
   out_786494535084995000[13] = 0;
   out_786494535084995000[14] = 0;
   out_786494535084995000[15] = 0;
   out_786494535084995000[16] = 0;
   out_786494535084995000[17] = 0;
   out_786494535084995000[18] = 0;
   out_786494535084995000[19] = 0;
   out_786494535084995000[20] = 0;
   out_786494535084995000[21] = 0;
   out_786494535084995000[22] = 1;
   out_786494535084995000[23] = 0;
   out_786494535084995000[24] = 0;
   out_786494535084995000[25] = 0;
   out_786494535084995000[26] = 0;
   out_786494535084995000[27] = 0;
   out_786494535084995000[28] = 0;
   out_786494535084995000[29] = 0;
   out_786494535084995000[30] = 0;
   out_786494535084995000[31] = 0;
   out_786494535084995000[32] = 0;
   out_786494535084995000[33] = 0;
   out_786494535084995000[34] = 0;
   out_786494535084995000[35] = 0;
   out_786494535084995000[36] = 0;
   out_786494535084995000[37] = 0;
   out_786494535084995000[38] = 0;
   out_786494535084995000[39] = 0;
   out_786494535084995000[40] = 0;
   out_786494535084995000[41] = 1;
   out_786494535084995000[42] = 0;
   out_786494535084995000[43] = 0;
   out_786494535084995000[44] = 0;
   out_786494535084995000[45] = 0;
   out_786494535084995000[46] = 0;
   out_786494535084995000[47] = 0;
   out_786494535084995000[48] = 0;
   out_786494535084995000[49] = 0;
   out_786494535084995000[50] = 0;
   out_786494535084995000[51] = 0;
   out_786494535084995000[52] = 0;
   out_786494535084995000[53] = 0;
}
void h_14(double *state, double *unused, double *out_869305805333252936) {
   out_869305805333252936[0] = state[6];
   out_869305805333252936[1] = state[7];
   out_869305805333252936[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7081556792712700097) {
   out_7081556792712700097[0] = 0;
   out_7081556792712700097[1] = 0;
   out_7081556792712700097[2] = 0;
   out_7081556792712700097[3] = 0;
   out_7081556792712700097[4] = 0;
   out_7081556792712700097[5] = 0;
   out_7081556792712700097[6] = 1;
   out_7081556792712700097[7] = 0;
   out_7081556792712700097[8] = 0;
   out_7081556792712700097[9] = 0;
   out_7081556792712700097[10] = 0;
   out_7081556792712700097[11] = 0;
   out_7081556792712700097[12] = 0;
   out_7081556792712700097[13] = 0;
   out_7081556792712700097[14] = 0;
   out_7081556792712700097[15] = 0;
   out_7081556792712700097[16] = 0;
   out_7081556792712700097[17] = 0;
   out_7081556792712700097[18] = 0;
   out_7081556792712700097[19] = 0;
   out_7081556792712700097[20] = 0;
   out_7081556792712700097[21] = 0;
   out_7081556792712700097[22] = 0;
   out_7081556792712700097[23] = 0;
   out_7081556792712700097[24] = 0;
   out_7081556792712700097[25] = 1;
   out_7081556792712700097[26] = 0;
   out_7081556792712700097[27] = 0;
   out_7081556792712700097[28] = 0;
   out_7081556792712700097[29] = 0;
   out_7081556792712700097[30] = 0;
   out_7081556792712700097[31] = 0;
   out_7081556792712700097[32] = 0;
   out_7081556792712700097[33] = 0;
   out_7081556792712700097[34] = 0;
   out_7081556792712700097[35] = 0;
   out_7081556792712700097[36] = 0;
   out_7081556792712700097[37] = 0;
   out_7081556792712700097[38] = 0;
   out_7081556792712700097[39] = 0;
   out_7081556792712700097[40] = 0;
   out_7081556792712700097[41] = 0;
   out_7081556792712700097[42] = 0;
   out_7081556792712700097[43] = 0;
   out_7081556792712700097[44] = 1;
   out_7081556792712700097[45] = 0;
   out_7081556792712700097[46] = 0;
   out_7081556792712700097[47] = 0;
   out_7081556792712700097[48] = 0;
   out_7081556792712700097[49] = 0;
   out_7081556792712700097[50] = 0;
   out_7081556792712700097[51] = 0;
   out_7081556792712700097[52] = 0;
   out_7081556792712700097[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_3393701557564291141) {
  err_fun(nom_x, delta_x, out_3393701557564291141);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6525324592803266903) {
  inv_err_fun(nom_x, true_x, out_6525324592803266903);
}
void pose_H_mod_fun(double *state, double *out_9151286745456408536) {
  H_mod_fun(state, out_9151286745456408536);
}
void pose_f_fun(double *state, double dt, double *out_8343874082406999130) {
  f_fun(state,  dt, out_8343874082406999130);
}
void pose_F_fun(double *state, double dt, double *out_3953690814815203097) {
  F_fun(state,  dt, out_3953690814815203097);
}
void pose_h_4(double *state, double *unused, double *out_3795290653571420264) {
  h_4(state, unused, out_3795290653571420264);
}
void pose_H_4(double *state, double *unused, double *out_3998768360417327801) {
  H_4(state, unused, out_3998768360417327801);
}
void pose_h_10(double *state, double *unused, double *out_2903431963768148397) {
  h_10(state, unused, out_2903431963768148397);
}
void pose_H_10(double *state, double *unused, double *out_3193504043246302910) {
  H_10(state, unused, out_3193504043246302910);
}
void pose_h_13(double *state, double *unused, double *out_7802322127374541928) {
  h_13(state, unused, out_7802322127374541928);
}
void pose_H_13(double *state, double *unused, double *out_786494535084995000) {
  H_13(state, unused, out_786494535084995000);
}
void pose_h_14(double *state, double *unused, double *out_869305805333252936) {
  h_14(state, unused, out_869305805333252936);
}
void pose_H_14(double *state, double *unused, double *out_7081556792712700097) {
  H_14(state, unused, out_7081556792712700097);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
