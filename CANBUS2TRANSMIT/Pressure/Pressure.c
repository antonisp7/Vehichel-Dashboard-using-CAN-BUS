#include "Pressure.h"
#include "stdio.h"

// function to sort the array in ascending order
void Array_sort_pressure(int *array , int k)
{
    // declare some local variables
    int i=0 , j=0 , temp=0;

    for(i=0 ; i<k ; i++)
    {
        for(j=0 ; j<k-1 ; j++)
        {
            if(array[j]>array[j+1])
            {
                temp        = array[j];
                array[j]    = array[j+1];
                array[j+1]  = temp;
            }
        }
    }
}


// function to calculate the median of the array
float Find_median_pressure(int array[] , int k)
{
   float median=0;

  // if number of elements are even
   if(k%2 == 0)
   median = (array[(k-1)/2] + array[k/2])/2.0;
   // if number of elements are odd
    else
    median = array[k/2];

    return median;
}

uint16_t Pressure_median(uint16_t pressure_value)
{
	    // declare some local variables
	    int i=0 ,k=25;
	    float median=0;

	    // declare two int arrays
	  	    int array_pressure[k];

	    for(i = 0; i < k; i++)
	    {
	    	array_pressure[i] = pressure_value;
	    }

	    // Sort the array in ascending order
	       Array_sort_pressure(array_pressure, k);

	       // Now pass the sorted array to calculate
	      //    the median of your array.
	          median = Find_median_pressure(array_pressure , k);

	 return(median);
}
