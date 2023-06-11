#include<iostream>
#include "hash.hpp"
#include "angle_table.hpp"
#include<math.h>
// direct memory hashing for unit vector search


#define number_of_stars_in_catalog 8874
#define angle_table_size 275876


int getClosest(float val1, float val2,  float target,int m1, int m2);
int unit_vector_hash(int star_id)
{

if(star_id >0 && star_id< 8875)
  return (star_id-1);
else
   return 0;  

}

int hash_index(float ang)
{
  int n= angle_table_size;
  float target = ang;
  // Corner cases
   if (target <= angle_table[0])
       return 0;
   if (target >= angle_table[n - 1])
       return n - 1;

   // Doing binary search
   int i = 0, j = n, mid = 0;
   while (i < j) {
       mid = (i + j) / 2;

       if (angle_table[mid] == target)
           return mid;

       /* If target is less than angle_tableay element,
           then search in left */
       if (target < angle_table[mid]) {

           // If target is greater than previous
           // to mid, return closest of two
           if (mid > 0 && target > angle_table[mid - 1])
               return getClosest(angle_table[mid - 1],
                                 angle_table[mid], target,mid-1,mid);

           /* Repeat for left half */
           j = mid;
       }

       // If target is greater than mid
       else {
           if (mid < n - 1 && target < angle_table[mid + 1])
               return getClosest(angle_table[mid],
                                 angle_table[mid + 1], target,mid,mid+1);
           // update i
           i = mid + 1;
       }
   }

   // Only single element left after search
   return mid;
}

int getClosest(float val1, float val2,float target,int m1, int m2)
{
    if (target - val1 >= val2 - target)
        return m2;
    else
        return m1;
}
