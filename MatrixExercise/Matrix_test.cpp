#include "Matrix.hpp"
#include <string>
#include <limits>

// See the comments in Main.cpp
// @http://www.boost.org/doc/libs/1_68_0/libs/test/doc/html/boost_test/adv_scenarios/shared_lib_customizations/entry_point.html
//#define BOOST_TEST_MODULE MatrixTestModule
//#define BOOST_TEST_DYN_LINK
//#define BOOST_TEST_NO_MAIN

// Only one of the following 3 includes may be active!

// For the header only version
//#include <boost/test/included/unit_test.hpp>
// For the static library version
//#include <boost/test/unit_test.hpp>
// For the dynamic library version
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE( MatrixConstructors)
	BOOST_AUTO_TEST_CASE( DefaultContructor)
	{
		std::string m0_as_string("Matrix<3,3>\n"
			"{\n"
			"0.000000,0.000000,0.000000,\n"
			"0.000000,0.000000,0.000000,\n"
			"0.000000,0.000000,0.000000,\n"
			"}");
		Matrix<double, 3,3 > m0;
		BOOST_CHECK_EQUAL( m0_as_string, m0.to_string());

		std::string m1_as_string("Matrix<3,3>\n"
			"{\n"
			"1.000000,1.000000,1.000000,\n"
			"1.000000,1.000000,1.000000,\n"
			"1.000000,1.000000,1.000000,\n"
			"}");
		Matrix<double, 3,3 > m1(1);
		BOOST_CHECK_EQUAL( m1_as_string, m1.to_string());
	}
	BOOST_AUTO_TEST_CASE( LinearConstructor )
	{
		std::string m0_as_string("Matrix<3,3>\n"
			"{\n"
			"1.000000,2.000000,3.000000,\n"
			"4.000000,5.000000,6.000000,\n"
			"7.000000,8.000000,9.000000,\n"
			"}");
		Matrix<double, 3,3 > m0{1,2,3,4,5,6,7,8,9};
		BOOST_CHECK_EQUAL( m0_as_string, m0.to_string());
	}
	BOOST_AUTO_TEST_CASE( ArrayListConstructor )
	{
		std::string m0_as_string("Matrix<3,3>\n"
			"{\n"
			"1.000000,2.000000,3.000000,\n"
			"4.000000,5.000000,6.000000,\n"
			"7.000000,8.000000,9.000000,\n"
			"}");
		Matrix<double, 3,3 > m0{{1,2,3},{4,5,6},{7,8,9}};
		BOOST_CHECK_EQUAL( m0_as_string, m0.to_string());
	}
	BOOST_AUTO_TEST_CASE( CopyConstructor )
	{
		std::string m0_as_string("Matrix<3,3>\n"
			"{\n"
			"1.000000,2.000000,3.000000,\n"
			"4.000000,5.000000,6.000000,\n"
			"7.000000,8.000000,9.000000,\n"
			"}");
		Matrix<double, 3,3 > m0{{1,2,3},{4,5,6},{7,8,9}};

		Matrix<double, 3,3 > m1 = m0;

		BOOST_CHECK_EQUAL( m0.to_string(),m1.to_string());

		BOOST_CHECK_EQUAL( m0,m1);
	}
BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( MatrixElementAccess)
BOOST_AUTO_TEST_CASE( At)
{
	Matrix<double, 3,3 > m0{{1,2,3},{4,5,6},{7,8,9}};

	BOOST_CHECK_THROW(m0.at(m0.getRows()+1), std::out_of_range);

	BOOST_CHECK_THROW(m0.at(m0.getRows(), m0.getColumns()+1), std::out_of_range);

	BOOST_CHECK_NO_THROW(m0[m0.getRows()+1]);
}
BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( MatrixOperators)
	BOOST_AUTO_TEST_CASE( AssignmentOperator)
	{
		Matrix<double, 3,3 > m0;
		Matrix<double, 3,3 > m1{{1,2,3},{4,5,6},{7,8,9}};
		m0 = m1;
		BOOST_CHECK_EQUAL( m0,m1);
	}
	BOOST_AUTO_TEST_CASE( ComparisonOperator)
	{
		Matrix<double, 3,3 > m0{{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m1{{1,2,3},{4,5,6},{7,8,9}};
		BOOST_CHECK_EQUAL( m0,m1);

		Matrix<double, 3,3 > m2{{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m3{{9,8,7},{6,5,4},{3,2,1}};
		BOOST_CHECK_NE( m2,m3);
	}
	BOOST_AUTO_TEST_CASE( ScalarMultiplication)
	{
		Matrix<double, 3,3 > m0 = {{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m1 = {{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m2{{1*2,2*2,3*2},{4*2,5*2,6*2},{7*2,8*2,9*2}};

		BOOST_CHECK_EQUAL( m2,m1*2);
		BOOST_CHECK_EQUAL( m0,m1);
		BOOST_CHECK_EQUAL( m2,m1*=2);
		BOOST_CHECK_EQUAL( m2,m1);
	}
	BOOST_AUTO_TEST_CASE( ScalarDivision)
	{
		Matrix<double, 3,3 > m0{{1*2,2*2,3*2},{4*2,5*2,6*2},{7*2,8*2,9*2}};
		Matrix<double, 3,3 > m1{{1*2,2*2,3*2},{4*2,5*2,6*2},{7*2,8*2,9*2}};
		Matrix<double, 3,3 > m2 = {{1,2,3},{4,5,6},{7,8,9}};

		BOOST_CHECK_EQUAL( m2,m1/2);
		BOOST_CHECK_EQUAL( m0,m1);
		BOOST_CHECK_EQUAL( m2,m1/=2);
		BOOST_CHECK_EQUAL( m2,m1);
	}
	BOOST_AUTO_TEST_CASE( MatrixAddition)
	{
		Matrix<double, 3,3 > m0 = {{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m1 = {{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m2{{1*2,2*2,3*2},{4*2,5*2,6*2},{7*2,8*2,9*2}};

		BOOST_CHECK_EQUAL( m2,m0+m1);
		BOOST_CHECK_EQUAL( m0,m1);
		BOOST_CHECK_EQUAL( m2,m1+=m0);
		BOOST_CHECK_EQUAL( m2,m1);
	}
	BOOST_AUTO_TEST_CASE( MatrixSubtraction)
	{
		Matrix<double, 3,3 > m0{{1*2,2*2,3*2},{4*2,5*2,6*2},{7*2,8*2,9*2}};
		Matrix<double, 3,3 > m1 = {{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m2 = {{1,2,3},{4,5,6},{7,8,9}};

		BOOST_CHECK_EQUAL( m2,m0-m1);
		BOOST_CHECK_EQUAL( m2,m0-=m1);
	}
	BOOST_AUTO_TEST_CASE( MatrixMatrixMultiplication)
	{
		Matrix<double, 3,3 > m0{{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m1{{30,36,42},{66,81,96},{102,126,150}};

		BOOST_CHECK_EQUAL( m1,m0*m0);
	}
	BOOST_AUTO_TEST_CASE( MatrixColumnVectorMultiplication)
	{
		Matrix<double, 3,3 > m0{{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,1 > m1{{{1}},{{2}},{{3}}};
		Matrix<double, 3,1 > m2{{{14}},{{32}},{{50}}};

		BOOST_CHECK_EQUAL( m2,m0*m1);
	}
	BOOST_AUTO_TEST_CASE( MatrixRowVectorMultiplication)
	{
		Matrix<double, 1,3 > m0;//{{1},{2},{3}};
		m0.at(0,0)= 1;
		m0.at(0,1)= 2;
		m0.at(0,2)= 3;
		Matrix<double, 3,1 > m1;//{{1},{2},{3}};
		m1.at(0,0)= 1;
		m1.at(1,0)= 2;
		m1.at(2,0)= 3;
		Matrix<double, 1,1 > m2;
		m2.at(0,0)=14;

		BOOST_CHECK_EQUAL( m2,m0*m1);
	}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( MatrixFunctions)
	BOOST_AUTO_TEST_CASE( MatrixTranspose)
	{
		// See https://en.wikipedia.org/wiki/Transpose
		Matrix<double, 3,3 > m0{{1,2,3},{4,5,6},{7,8,9}};
		Matrix<double, 3,3 > m1{{9,8,7},{6,5,4},{3,2,1}};

		BOOST_CHECK_EQUAL( m0,m0.transpose().transpose());
		BOOST_CHECK_EQUAL( (m0+m1).transpose(), m0.transpose()+ m1.transpose());
		BOOST_CHECK_EQUAL( (m0*4.0).transpose(),m0.transpose()*4.0);
	}
	BOOST_AUTO_TEST_CASE( MatrixIdentity)
	{
		Matrix<double, 3,3 > m0{{1,0,0},{0,1,0},{0,0,1}};
		Matrix<double, 3,3 > m1{{1,2,3},{4,5,6},{7,8,9}};

		BOOST_CHECK_EQUAL( m0,m1.identity());
		BOOST_CHECK_EQUAL( m1,m1*m1.identity());
		BOOST_CHECK_EQUAL( m1,m1.identity()*m1);

	}
	// @See https://www.boost.org/doc/libs/1_68_0/libs/test/doc/html/boost_test/testing_tools/extended_comparison/floating_point.html
	BOOST_AUTO_TEST_CASE( MatrixGauss, * boost::unit_test::tolerance(0.00001))
	{
		Matrix<double, 3,4> m0{{0,1,1,5},{3,2,2,13},{1,-1,3,8}};
		Matrix<double, 3,4> m1(m0.gauss());

		double c = m1.at(2,3);
		double b = m1.at(1,3) - m1.at(1,2) * c;
		double a = m1.at(0,3) - m1.at(0,1) * b - m1.at(0,2) * c;

		BOOST_TEST(a == 1.0);
		BOOST_TEST(b == 2.0);
		BOOST_TEST(c == 3.0);
	}

//	BOOST_AUTO_TEST_CASE( MatrixGauss2, * boost::unit_test::tolerance(0.00001))
//	{
//		Matrix<double, 3,4> m0{{1,3,1,9},{1,1,-1,1},{3,11,5,35}};
//		Matrix<double, 3,4> m1(m0.gauss());
//
//		double c = m1.at(2,3);
//		double b = m1.at(1,3) - m1.at(1,2) * c;
//		double a = m1.at(0,3) - m1.at(0,1) * b - m1.at(0,2) * c;
//
//		BOOST_TEST(a == -3.0);
//		BOOST_TEST(b == 4.0);
//		BOOST_TEST(c == 0);
//	}

	BOOST_AUTO_TEST_CASE( MatrixGaussJordan)
	{
		Matrix<double, 3,4> m0{{0,1,1,5},{3,2,2,13},{1,-1,3,8}};
		Matrix<double, 3,4> m1{{1,0,0,1},{0,1,0,2},{0,0,1,3}};
		//BOOST_CHECK_EQUAL(m0.gaussJordan(),m1);
		BOOST_CHECK_EQUAL( true, equals(m0.gaussJordan(),m1,std::numeric_limits<double>::epsilon(),100));
	}

	BOOST_AUTO_TEST_CASE( MatrixSolve)
	{
		Matrix<double, 3,4> m0{{0,1,1,5},{3,2,2,13},{1,-1,3,8}};
		Matrix<double, 3, 1> m1{{{1}},{{2}},{{3}}};

		// The following test fails because of rounding errors, use equals instead.
		//BOOST_CHECK_EQUAL(m0.solve(),m1);

		BOOST_CHECK_EQUAL( true, equals(m0.solve(),m1,std::numeric_limits<double>::epsilon(),100));
	}
	BOOST_AUTO_TEST_CASE( MatrixInverse)
	{
		Matrix<double, 3,3> m0{{1,2,0},{1,0,1},{2,2,2}};

		BOOST_CHECK_EQUAL( m0.identity(),m0*m0.inverse());
		BOOST_CHECK_EQUAL( m0.identity(),m0.inverse()*m0);

		Matrix<double, 3,3> m1{{1,2,3},{0,1,5},{5,6,0}};

		// The following tests fail because of rounding errors, use equals instead.
		// BOOST_CHECK_EQUAL( m1.identity(),m1*m1.inverse());
		// BOOST_CHECK_EQUAL( m1.identity(),m1.inverse()*m1);

		BOOST_CHECK_EQUAL( true, equals(m1.identity(),m1*m1.inverse(),std::numeric_limits<double>::epsilon(),100));
		BOOST_CHECK_EQUAL( true, equals(m1.identity(),m1.inverse()*m1,std::numeric_limits<double>::epsilon(),100));
	}
	BOOST_AUTO_TEST_CASE( MatrixColumnVectorEquality)
	{
		Matrix<double, 3,1 > m0{{{1}},{{2}},{{3}}};
		Matrix<double, 3,1 > m1{{{1}},{{2}},{{3}}};

		BOOST_CHECK_EQUAL( true, equals(m0,m1/*,std::numeric_limits<double>::epsilon()*/));
	}
	BOOST_AUTO_TEST_CASE( MatrixRowVectorEquality)
	{
		Matrix<double, 1,3 > m0{{1,2,3}};
		Matrix<double, 1,3 > m1{{1,2,3}};

		BOOST_CHECK_EQUAL( true, equals(m0,m1/*,std::numeric_limits<double>::epsilon()*/));
	}
	BOOST_AUTO_TEST_CASE( MatrixEquality)
	{
		Matrix<double, 3,3 > m0{{1,2,3}, {4,5,6}, {7,8,9}};
		Matrix<double, 3,3 > m1{{1,2,3}, {4,5,6}, {7,8,9}};

		BOOST_CHECK_EQUAL( true, equals(m0,m1/*,std::numeric_limits<double>::epsilon()*/));
	}
BOOST_AUTO_TEST_SUITE_END()
