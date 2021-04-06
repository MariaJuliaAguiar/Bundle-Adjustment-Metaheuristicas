//  benchmarkFactory.cc
//
//  Author:
//       Ahmad Dajani <eng.adajani@gmail.com>
//
//  Copyright (c) 2020 Ahmad Dajani
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "benchmarkFactory.hpp"
#include "benchmark/fob.hpp"
BenchmarkFactory::BenchmarkFactory() {

	
	int teste;
	//Register("fob", &fob::Create);
}

BenchmarkFactory::~BenchmarkFactory() {
	factoryMap_m.clear();
}

BenchmarkFactory *BenchmarkFactory::Get() {
	static BenchmarkFactory instance;
	return &instance;
}

BenchmarkFactory & BenchmarkFactory::operator=(const BenchmarkFactory &) {
	return *this;
}

void BenchmarkFactory::Register(const std::string &benchmarkName,
	BenchmarkCreateMethod benchmarkCreateMethod) {
	factoryMap_m[benchmarkName] = benchmarkCreateMethod;
	
}

Benchmark* BenchmarkFactory::CreateBenchmark(const std::string &benchmarkName,  std::vector<double>lb, std::vector<double>up) {

	
	return fob::Create(lb, up);
}
