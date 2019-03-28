#include <cassert>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <utility>
#include <iomanip>

/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( T value)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) = value;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< T >& aList)
{
	// Check the arguments
	assert( aList.size() == M * N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column, ++row_iter)
		{
			matrix.at( row).at( column) = *row_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< std::initializer_list< T > >& aList)
{
	// Check the arguments, the static assert assures that there is at least 1 M and 1 N!
	assert( aList.size() == M && (*aList.begin()).size() == N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
	{
		auto column_iter = (*row_iter).begin();
		for (std::size_t column = 0; column < (*row_iter).size(); ++column, ++column_iter)
		{
			matrix.at( row).at( column) = *column_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const Matrix< T, M, N >& aMatrix) :
				matrix( aMatrix.matrix)
{
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex)
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex) const
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
T& Matrix< T, M, N >::at( 	std::size_t aRowIndex,
							std::size_t aColumnIndex)
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const T& Matrix< T, M, N >::at( std::size_t aRowIndex,
								std::size_t aColumnIndex) const
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex)
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex) const
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator=( const Matrix< T, M, N >& rhs)
{
	if (this != &rhs)
	{
		matrix = rhs.matrix;
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
bool Matrix< T, M, N >::operator==( const Matrix< T, M, N >& rhs) const
{
	return matrix == rhs.matrix;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator*=( const T2& scalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) *= scalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator*( const T2& scalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result *= scalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator/=( const T2& aScalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) /= aScalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator/( const T2& aScalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result /= aScalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator+=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] += rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator+( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result += rhs;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator-=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] -= rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator-( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result -= rhs;
}
/**
 * (M, N) * (N, P) -> (M, P)
 */
template< typename T, std::size_t M, std::size_t N >
template< std::size_t columns >
Matrix< T, M, columns > Matrix< T, M, N >::operator*( const Matrix< T, N, columns >& rhs) const
{
	// Create n * p matrix
	Matrix< T, M, columns> result;
	// Loop through the rows of the input matrix
	for(std::size_t i = 0; i < this->getRows(); ++i)
	{
		// Loop through the columns of the output matrix
		for(std::size_t j = 0; j < rhs.getColumns(); ++j)
		{
			// Loop through the columns of the input matrix
			for(std::size_t k=0; k < this->getColumns(); ++k)
			{
				// Multiply the values and add them in the correct field
				result[i][j] += this->at(i, k) * rhs.at(k, j);
			}
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, N, M > Matrix< T, M, N >::transpose() const
{
	Matrix< T, N, M > result;
	for(std::size_t i = 0; i < this->getRows(); ++i)
	{
		for(std::size_t j = 0; j < this->getColumns(); ++j)
		{
			result[j][i] = this->at(i,j);
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::identity() const
{
	Matrix< T, N, M > result;
	for(std::size_t i = 0; i < this->getRows(); i++)
	{
		for(std::size_t j = 0; j < this->getColumns(); j++)
		{
			// Set all values to 0
			result[i][j] = 0;
			// Except the diagonal
			if(i == j)
			{
				result[i][j] = 1;
			}
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::gauss() const
{
	Matrix< T, M, N > result;
	// Copy the current matrix to result
	result = *this;
	// Check that the input matrix has the right form
	static_assert( M == N - 1);

	for(std::size_t currentRow = 0, pivotColumn = 0; currentRow < M, pivotColumn < N - 1; currentRow++, pivotColumn++)
	{
		// Swap the row with the highest value to the top.
		std::size_t maxValueRow = currentRow;
		for(std::size_t i = currentRow; i < M; i++)
		{
			if(result[i][pivotColumn] > result[maxValueRow][pivotColumn])
			{
				maxValueRow = i;
			}
		}
		std::swap(result[maxValueRow], result[currentRow]);
		// 1. Divide the first number on the row by itself to get 1
		std::size_t nonZeroColumn = pivotColumn;
		T divideValue = result[currentRow][nonZeroColumn];
		for(std::size_t divideColumn = 0; divideColumn < N; divideColumn++)
		{
			result[currentRow][divideColumn] = result[currentRow][divideColumn] / divideValue;
		}
		// 2. Multiply every other non-zero row by a multiple of the first row to get 0
		// Loop through the rows
		for(std::size_t zeroRow = currentRow + 1; zeroRow < M; zeroRow++)
		{
			// Get the first number from the row if it is non-zero
			T currentRowValue = result[zeroRow][pivotColumn];
			// Multiply the first number by a multiple of the first row and subtract the multiplied value from the row
			for(std::size_t multiplyColumn = 0; multiplyColumn < N; multiplyColumn++)
			{
				result[zeroRow][multiplyColumn] -= (result[currentRow][multiplyColumn] * currentRowValue);
			}
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::gaussJordan() const
{
	Matrix<T, M, N> result;
	result = this->gauss();
	for(std::size_t currentRow = M - 1, pivotColumn = N - 2; currentRow >= 0, pivotColumn > 0; currentRow--, pivotColumn--)
	{
		int zeroRow = currentRow - 1;
		// 2. Multiply every other non-zero row by a multiple of the first row to get 0
		for(int currentZeroRow = zeroRow; currentZeroRow >= 0; currentZeroRow--)
		{
			// Get the first number from the row if it is non-zero
			T currentRowValue = result[currentZeroRow][pivotColumn];
			// Multiply the first number by a multiple of the first row and subtract the multiplied value from the row
			for(std::size_t multiplyColumn = N; multiplyColumn-- > 0;)
			{
				result[currentZeroRow][multiplyColumn] -= (result[currentRow][multiplyColumn] * currentRowValue);
			}
		}
	}
	return result;
}
/**
 *

 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, 1 > Matrix< T, M, N >::solve() const
{
	Matrix < T, M, 1 > result;
	Matrix < T, M, N > gaussResult;
	// Get the Gauss-Jordan result
	gaussResult = this->gaussJordan();
	// Get the answers from the right column
	for(std::size_t currentRow = 0; currentRow < M; currentRow++)
	{
		result[currentRow][0] = gaussResult[currentRow][N - 1];
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::inverse() const
{
  if(M != N)
  {
    throw std::logic_error("Can't inverse a matrix with non-equal M and N.");
  }
  
	// Copy the original matrix
	Matrix< T, N, M > result = *this;
	// Get the identity matrix
	Matrix < T, M, N > identityMatrix = result.identity();
	// Gauss eliminate the matrices
	for(std::size_t currentRow = 0, pivotColumn = 0; currentRow < M, pivotColumn < N; currentRow++, pivotColumn++)
	{
		// Swap the row with the highest value to the top.
		std::size_t maxValueRow = currentRow;
		for(std::size_t i = currentRow; i < M; i++)
		{
			if(result[i][pivotColumn] > result[maxValueRow][pivotColumn])
			{
				maxValueRow = i;
			}
		}
		std::swap(result[maxValueRow], result[currentRow]);
		std::swap(identityMatrix[maxValueRow], identityMatrix[currentRow]);
		// 1. Divide the first number on the row by itself to get 1
		T divideValue = result[currentRow][pivotColumn];
		T divideIdentityValue = identityMatrix[currentRow][pivotColumn];
		for(std::size_t divideColumn = 0; divideColumn < N; divideColumn++)
		{
			result[currentRow][divideColumn] = result[currentRow][divideColumn] / divideValue;
			identityMatrix[currentRow][divideColumn] = identityMatrix[currentRow][divideColumn] / divideValue;
		}
		// 2. Multiply every other non-zero row by a multiple of the first row to get 0
		// Loop through the rows
		for(std::size_t zeroRow = currentRow + 1; zeroRow < M; zeroRow++)
		{
			// Get the first number from the row if it is non-zero
			T currentRowValue = result[zeroRow][pivotColumn];
			T currentIdentityRowValue = identityMatrix[zeroRow][pivotColumn];
			// Multiply the first number by a multiple of the first row and subtract the multiplied value from the row
			for(std::size_t multiplyColumn = 0; multiplyColumn < N; multiplyColumn++)
			{
				result[zeroRow][multiplyColumn] -= (result[currentRow][multiplyColumn] * currentRowValue);
				identityMatrix[zeroRow][multiplyColumn] -= (identityMatrix[currentRow][multiplyColumn] * currentRowValue);
			}
		}
	}
	// Jordan eliminate the matrices
	for(int currentRow = M - 1, pivotColumn = N - 1; currentRow > 0, pivotColumn >= 0; currentRow--, pivotColumn--)
	{
		int zeroRow = currentRow - 1;
		// 2. Multiply every other non-zero row by a multiple of the first row to get 0
		for(int currentZeroRow = zeroRow; currentZeroRow >= 0; currentZeroRow--)
		{
			// Get the first number from the row if it is non-zero
			T currentRowValue = result[currentZeroRow][pivotColumn];
			// Multiply the first number by a multiple of the first row and subtract the multiplied value from the row
			for(std::size_t multiplyColumn = N; multiplyColumn-- > 0;)
			{
				result[currentZeroRow][multiplyColumn] -= (result[currentRow][multiplyColumn] * currentRowValue);
				identityMatrix[currentZeroRow][multiplyColumn] -= (identityMatrix[currentRow][multiplyColumn] * currentRowValue);
			}
		}
	}
	return identityMatrix;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::string Matrix< T, M, N >::to_string() const
{
	std::string result = "Matrix<" + std::to_string( N) + "," + std::to_string( M) + ">\n{\n";
	for (std::size_t i = 0; i < M; ++i)
	{
		for (std::size_t j = 0; j < N; ++j)
		{
			result += std::to_string( matrix[i][j]) + ",";
		}
		result += "\n";
	}
	result += "}";
	return result;
}
/**
 *
 */
template< typename T, const std::size_t N >
bool equals(	const Matrix< T, 1, N >& lhs,
				const Matrix< T, 1, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	// Row vector equality
	bool lReturn = true;
	for(std::size_t currentColumn = 0; currentColumn < N - 1; currentColumn++)
	{
		T diff = lhs[0][currentColumn] - rhs[0][currentColumn];
		if((diff > (aPrecision * aFactor)) || (-diff > (aPrecision  * aFactor)))
		{
			lReturn = false;
			break;
		}
	}
	return lReturn;
}
/**
 *
 */
template< typename T, const std::size_t M >
bool equals(	const Matrix< T, M, 1 >& lhs,
				const Matrix< T, M, 1 >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	// Column vector equality
	bool lReturn = true;
	for(std::size_t currentRow = 0; currentRow < M; currentRow++)
	{
		T diff = lhs[currentRow][0] - rhs[currentRow][0];
		if((diff > (aPrecision  * aFactor)) || (-diff > (aPrecision * aFactor)))
		{
			lReturn = false;
			break;
		}
	}
	return lReturn;
}
/**
 *
 */
template< typename T, const std::size_t M, const std::size_t N >
bool equals(	const Matrix< T, M, N >& lhs,
				const Matrix< T, M, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	// Full matrix equality
	bool lReturn = true;
	for(std::size_t currentRow = 0; currentRow < M; currentRow++)
	{
		for(std::size_t currentColumn = 0; currentColumn < N; currentColumn++)
		{
			T diff = lhs[currentRow][currentColumn] - rhs[currentRow][currentColumn];
			if((diff > (aPrecision * aFactor)) || (-diff > (aPrecision * aFactor)))
			{
				lReturn = false;
				break;
			}
		}
	}
	return lReturn;
}
