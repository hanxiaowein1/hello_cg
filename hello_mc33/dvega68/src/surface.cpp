/*
	File: surface.cpp
	Programmed by: David Vega - dvega@uc.edu.ve
	August 2019
	February 2020
	August 2020
	July 2021
*/

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <format>
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress(double percentage) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

#include "../include/MC33.h"

void surface::save_as_obj(std::string filename)
{
	std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

	std::cout << "start save vertex to obj" << std::endl;
	for(unsigned int i = 0; i < this->nV; i++)
	{
		printProgress(double(i) / nV);
		file << "v " << this->V[i].v[0] << " " << this->V[i].v[1] << " " << this->V[i].v[2] << std::endl;
	}
	std::cout << std::endl;
	// for(auto vertex_iterator = this->V.begin(); vertex_iterator != this->V.end(); vertex_iterator++)
	// {
	// 	int index = std::distance(this->V.begin(), vertex_iterator);
	// 	printProgress(double(index) / double(this->V.size()));
	// 	// file << std::format("v {} {} {}", vertex.v[0], vertex.v[1], vertex.v[2]) << std::endl;
	// 	file << "v " << vertex_iterator->v[0] << " " << vertex_iterator->v[1] << " " << vertex_iterator->v[2] << std::endl;
	// }

	// for(const auto &triangle: this->T)
	std::cout << "start save triangle to obj" << std::endl;
	for(unsigned int i = 0; i < this->nT; i++)
	{
		printProgress(double(i) / nT);
		file << "f " << this->T[i].v[0] + 1 << " " << this->T[i].v[1] + 1 << " " << this->T[i].v[2] + 1 << std::endl;
	}
	std::cout << std::endl;
	// for(auto triangle_iterator = this->T.begin(); triangle_iterator != this->T.end(); triangle_iterator++)
	// {
	// 	int index = std::distance(this->T.begin(), triangle_iterator);
	// 	printProgress(double(index) / double(this->T.size()));
	// 	// file << std::format("f {} {} {}", triangle.v[0] + 1, triangle.v[1] + 1, triangle.v[2] + 1) << std::endl;
	// 	file << "f " << triangle_iterator->v[0] + 1 << " " << triangle_iterator->v[1] + 1 << " " << triangle_iterator->v[2] + 1 << std::endl;
	// }
	file.close();
}


void* surface::get_vertices_pointer()
{
	return &(this->V[0]);
}

void* surface::get_triangle_index_pointer()
{
	return &(this->T[0]);
}


MC33_real surface::get_isovalue() { return iso;}

unsigned int surface::get_num_vertices() { return nV; }

unsigned int surface::get_num_triangles() { return nT; }

surface::surface() : nV(0), nT(0) {}

void surface::clear() {
	T.clear();
	V.clear();
	N.clear();
	color.clear();
	nV = nT = 0;
}

void surface::adjustvectorlenght() {
	T.resize(nT);
	T.shrink_to_fit();
	V.resize(nV);
	V.shrink_to_fit();
	N.resize(nV);
	N.shrink_to_fit();
	color.shrink_to_fit();
}


const unsigned int *surface::getTriangle(unsigned int n) { return (n < nT? T[n].v: 0); }

const MC33_real *surface::getVertex(unsigned int n) { return (n < nV? V[n].v: 0); }

const float *surface::getNormal(unsigned int n) { return (n < nV? N[n].v: 0); }

void surface::flipNormals() {
	unsigned int n = 3*nV;
	float *v = N[0].v;
	for (unsigned int i = 0; i != n; i++)
		v[i] = -v[i];
}

void surface::flipTriangles() {
	for (auto &t : T)
		std::swap(t.v[0], t.v[1]);
}

void surface::setColor(unsigned int n, unsigned char *pcolor) {
	if (n < nV)
		color[n] = *(reinterpret_cast<int*>(pcolor));
}

const unsigned char* surface::getColor(unsigned int n) {
	return (n < nV? reinterpret_cast<unsigned char*>(&color[n]): 0);
}


#if MC33_double_precision
#define MC33_surf_magic_num 0x6575732e //".sud"
#define MC33_surf_magic_nu2 0x7075732e
#define MC33_real2 float
#else
#define MC33_surf_magic_num 0x7075732e //".sup"
#define MC33_surf_magic_nu2 0x6575732e
#define MC33_real2 double
#endif

int surface::save_bin(const char *filename) {
	std::ofstream out(filename, std::ios::binary);
	if (!out)
		return -1;

	int n = MC33_surf_magic_num;
	out.write(reinterpret_cast<char*>(&n),sizeof(int));
	out.write(reinterpret_cast<char*>(&iso),sizeof(MC33_real));
	out.write(reinterpret_cast<char*>(&nV),sizeof(int));
	out.write(reinterpret_cast<char*>(&nT),sizeof(int));

	out.write(reinterpret_cast<char*>(&T[0]),3*sizeof(int)*nT);
	out.write(reinterpret_cast<char*>(&V[0]),3*sizeof(MC33_real)*nV);
	out.write(reinterpret_cast<char*>(&N[0]),3*sizeof(float)*nV);
	out.write(reinterpret_cast<char*>(&color[0]),sizeof(int)*nV);
	return (out.good()? 0: -1);
}

int surface::read_bin(const char *filename) {
	std::ifstream in(filename, std::ios::binary);
	if (!in)
		return -1;
	int n;
	in.read(reinterpret_cast<char*>(&n),sizeof(int));
	if (n == MC33_surf_magic_num || n == MC33_surf_magic_nu2)
	{
		if (n == MC33_surf_magic_nu2) {
			MC33_real2 t;
			in.read(reinterpret_cast<char*>(&t),sizeof(MC33_real2));
			iso = t;
		} else
			in.read(reinterpret_cast<char*>(&iso),sizeof(MC33_real));
		in.read(reinterpret_cast<char*>(&nV),sizeof(int));
		in.read(reinterpret_cast<char*>(&nT),sizeof(int));
		adjustvectorlenght();
		in.read(reinterpret_cast<char*>(&T[0]),3*sizeof(int)*nT);
		if (n == MC33_surf_magic_nu2) {
			for (unsigned int j = 0; j != nV; j++) {
				MC33_real2 t[3];
				in.read(reinterpret_cast<char*>(t),3*sizeof(MC33_real2));
				for (int i = 0; i != 3; i++)
					V[j].v[i] = t[i];
			}
		} else
			in.read(reinterpret_cast<char*>(V[0].v),3*sizeof(MC33_real)*nV);
		in.read(reinterpret_cast<char*>(&N[0]),3*sizeof(int)*nV);
		in.read(reinterpret_cast<char*>(&color[0]),sizeof(int)*nV);
	} else
		return -1;
	return (in.good()? 0: -1);
}
#undef MC33_surf_magic_num
#undef MC33_surf_magic_nu2
#undef MC33_real2

#if MC33_double_precision
#define MC33_prec 11
#else
#define MC33_prec 6
#endif
int surface::save_txt(const char* filename) {
	std::ofstream out(filename);
	adjustvectorlenght();
	if (!out)
		return -1;

	out << "isovalue: ";
	out.setf(std::ios_base::scientific, std::ios_base::floatfield);
	out.precision(MC33_prec);
	out << iso << "\n\nVERTICES:\n" << nV << "\n\n";
	out.setf(std::ios_base::fixed, std::ios_base::basefield);
	out.precision(MC33_prec);
	for (const auto &r: V)
		out << std::setw(7 + MC33_prec) << r.v[0] << std::setw(8 + MC33_prec) << r.v[1] << std::setw(8 + MC33_prec) << r.v[2] << std::endl;

	out << "\n\nTRIANGLES:\n" << nT << "\n\n";
	for (const auto &t: T)
		out << std::setw(8) << t.v[0] << " " << std::setw(8) << t.v[1] << " "  << std::setw(8) << t.v[2] << std::endl;

	out << "\n\nNORMALS:\n";
	out.precision(5);
	for (const auto &r: N)
		out << std::setw(12) << r.v[0] << std::setw(13) << r.v[1] << std::setw(13) << r.v[2] << std::endl;

	out << "\n\nCOLORS:\n";
	for (const auto &c: color)
		out << c << std::endl;
	out << "\nEND\n";
	return (out.good()? 0: -1);
}

#undef MC33_prec
