/*
 * This file was autogenerated by TinyMPC on Tue Nov 14 05:15:07 2023
 */

#include <tinympc/tiny_data_workspace.hpp>

#ifdef __cplusplus
extern "C" {
#endif

/* User settings */
TinySettings settings = {
	(tinytype)0.0010000000474975,	// primal tolerance
	(tinytype)0.0010000000474975,	// dual tolerance
	100,		// max iterations
	1,		// iterations per termination check
	1,		// enable state constraints
	1		// enable input constraints
};

/* Matrices that must be recomputed with changes in time step, rho */
TinyCache cache = {
	(tinytype)0.1000000000000000,	// rho (step size/penalty)
	(tiny_MatrixNuNx() << (tinytype)-2.9121762085735625,(tinytype)-4.8173683645577148,(tinytype)44.3538693717722552,(tinytype)19.7167443148291497).finished(),	// Kinf
	(tiny_MatrixNxNx() << (tinytype)1670.7543004350841329,(tinytype)1318.5384369756732212,(tinytype)-6761.4864655047085762,(tinytype)-3032.1313994557076512,(tinytype)1318.5384369756775413,(tinytype)1485.1549227986477035,(tinytype)-8118.8844330037645705,(tinytype)-3643.6820433256280012,(tinytype)-6761.4864655047258566,(tinytype)-8118.8844330037527470,(tinytype)50800.4058220436854754,(tinytype)22576.6325867217601626,(tinytype)-3032.1313994557181104,(tinytype)-3643.6820433256284559,(tinytype)22576.6325867217819905,(tinytype)10104.8716227756867738).finished(),	// Pinf
	(tiny_MatrixNuNu() << (tinytype)0.8396930621947403).finished(),	// Quu_inv
	(tiny_MatrixNxNx() << (tinytype)1.0002174920524525,(tinytype)0.0434984928022743,(tinytype)0.0001105937714669,(tinytype)0.0221197145976064,(tinytype)0.0103597788244895,(tinytype)1.0719559010593862,(tinytype)0.0001829459819133,(tinytype)0.0365907849332197,(tinytype)-0.0032901803531126,(tinytype)-0.6580371303755368,(tinytype)0.9985761203568297,(tinytype)-0.2847882924192180,(tinytype)-0.0014724447036229,(tinytype)-0.2944820540050069,(tinytype)0.0092520987200908,(tinytype)0.8505000890797787).finished(),	// AmBKt
	(tiny_MatrixNxNu() << (tinytype)0.0000011629559782,(tinytype)0.0000018195428124,(tinytype)-0.0000113319731909,(tinytype)-0.0000050820250905).finished(),	// coeff_d2p
};

/* Problem variables */
TinyWorkspace work = {
	(tiny_MatrixNxNh() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// x
	(tiny_MatrixNuNhm1() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// u
	(tiny_MatrixNxNh() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// q
	(tiny_MatrixNuNhm1() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// r
	(tiny_MatrixNxNh() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// p
	(tiny_MatrixNuNhm1() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// d
	(tiny_MatrixNxNh() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// v
	(tiny_MatrixNxNh() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// vnew
	(tiny_MatrixNuNhm1() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// z
	(tiny_MatrixNuNhm1() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// znew
	(tiny_MatrixNxNh() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// g
	(tiny_MatrixNuNhm1() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// y
	(tinytype)0.0000000000000000,	// state primal residual
	(tinytype)0.0000000000000000,	// input primal residual
	(tinytype)0.0000000000000000,	// state dual residual
	(tinytype)0.0000000000000000,	// input dual residual
	0,	// solve status
	0,	// solve iteration
	(tiny_VectorNx() << (tinytype)10.0999999999999996,(tinytype)1.1000000000000001,(tinytype)10.0999999999999996,(tinytype)1.1000000000000001).finished(),	// Q
	(tiny_VectorNx() << (tinytype)10.0999999999999996,(tinytype)1.1000000000000001,(tinytype)10.0999999999999996,(tinytype)1.1000000000000001).finished(),	// Qf
	(tiny_VectorNu() << (tinytype)1.1000000000000001).finished(),	// R
	(tiny_MatrixNxNx() << (tinytype)1.0000000000000000,(tinytype)0.0100000000000000,(tinytype)0.0000223300834033,(tinytype)0.0000000744303797,(tinytype)0.0000000000000000,(tinytype)1.0000000000000000,(tinytype)0.0044662105765102,(tinytype)0.0000223300834033,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)1.0002605176397052,(tinytype)0.0100008683544304,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0521057900592854,(tinytype)1.0002605176397052).finished(),	// Adyn
	(tiny_MatrixNxNu() << (tinytype)0.0000746836856273,(tinytype)0.0149367653901618,(tinytype)0.0000379763323185,(tinytype)0.0075955962185547).finished(),	// Bdyn
	(tiny_MatrixNuNhm1() << (tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000).finished(),	// u_min
	(tiny_MatrixNuNhm1() << (tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000).finished(),	// u_max
	(tiny_MatrixNxNh() << (tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000,(tinytype)-5.0000000000000000).finished(),	// x_min
	(tiny_MatrixNxNh() << (tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000,(tinytype)5.0000000000000000).finished(),	// x_max
	(tiny_MatrixNxNh() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// Xref
	(tiny_MatrixNuNhm1() << (tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000,(tinytype)0.0000000000000000).finished(),	// Uref
	(tiny_VectorNu() << (tinytype)0.0000000000000000).finished()	// Qu
};

TinySolver tiny_data_solver = {&settings, &cache, &work};

#ifdef __cplusplus
}
#endif

