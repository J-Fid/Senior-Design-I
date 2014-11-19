#include <stdio.h>

#include "../include/vertex.h"
#include <assert.h>

/*int main(){

	int arr[] = {0, 0, 10, 20, 100, 350};	
	
	Vertex* v1 = new Vertex(arr[0] , arr[1]);
	Vertex* v2 = new Vertex(arr[2] , arr[3]);
	Vertex* v3 = new Vertex(arr[4] , arr[5]);

	printf("-------------------------------------------\n");
	printf("\tTesting Vertex class...\n");
	printf("-------------------------------------------\n\n");

	
	printf("-------------------------------------------\n");
	printf("\tTesting getPoint() method...\n");
	printf("-------------------------------------------\n\n");
	
	//the getPoint method is being tested by comparing the x, y-coordinates with the array of int values declared above. 
	int point[2];	

	v1->getPoints(point);
	printf("Correct output is -> x: %i, y: %i\nv1 output is -> x: %i, y: %i\n\n", 0, 0, point[0], point[1]);

	v2->getPoints(point);
	printf("Correct output is -> x: %i, y: %i\nv2 output is -> x: %i, y: %i\n\n", 10, 20, point[0], point[1]);

	v3->getPoints(point);
	printf("Correct output is -> x: %i, y: %i\nv3 output is -> x: %i, y: %i\n\n", 100, 350, point[0], point[1]);


	printf("-----------------------------------------------------\n");
	printf("\tTesting SetNextVertex & getNextVertex() methods...\n");
	printf("-----------------------------------------------------\n\n");

	v1->setNextVertex(v2);
	v2->setNextVertex(v3);
	v3->setNextVertex(v1);

	
	v1->getNextVertex()->getPoints(point);
	printf("V1's next vertex is V2. Point output should be %i, %i. \nThe actual output is %i, %i\n ", 10, 20, point[0], point[1]);

	v2->getNextVertex()->getPoints(point);
	printf("V2's next vertex is V3. Point output should be %i, %i. \nThe actual output is %i, %i\n", 100, 350, point[0], point[1]);

	v3->getNextVertex()->getPoints(point);
	printf("V3's next vertex is V1. Point output should be %i, %i. \nThe actual output is %i, %i\n", 0, 0, point[0], point[1]);
	
	//getVisited() - setVisited()
	printf("-------------------------------------------------\n");
	printf("\tTesting Visiting Methods...\n");
	printf("-------------------------------------------------\n");

	v1->setVisited(2);
	v2->setVisited(1);


	printf("v1 visited (0 for not, 1 for referenced, 2 for completed) should be 2: %i\n", v1->getVisited());
	printf("v2 visited (0 for not, 1 for referenced, 2 for completed) should be 1: %i\n", v2->getVisited());
	printf("v3 visited (0 for not, 1 for referenced, 2 for completed) should be 0: %i\n", v3->getVisited());

	printf("-------------------------------------------\n");
	printf("\t\tEnd of Tests...\n");
	printf("-------------------------------------------\n");

	return 0;
	test_Vertex();
}*/

int test_Vertex(Vertex* v1)
{
	int point[2];

	v1->getPoints(point);
	
	assert(point[0] == 5);
	assert(point[1] == 3);
	assert(v1->getVisited() == 0);
	assert(v1->getNextVertex() == NULL);
	
	return 0;
}

int test_getPoints(Vertex* v1)
{
	int point[2];
	
	v1->getPoints(point);
	
	assert(point[0] == 5);
	assert(point[1] == 3);
	
	return 0;
}

int test_setPoints(Vertex* v1)
{
	int point[2];
	
	v1->setPoints(6, 7);
	v1->getPoints(point);
	
	assert(point[0] == 6);
	assert(point[1] == 7);	
	
	return 0;
}

int	test_getVisited(Vertex* v1)
{
	assert(v1->getVisited() == 0);
	
	return 0;
}

int	test_setVisited(Vertex* v1)
{
	v1->setVisited(2);
	
	assert(v1->getVisited() == 2);
	
	return 0;
}

int	test_getNextVertex(Vertex* v1)
{
	assert(v1->getNextVertex() == NULL);
	
	return 0;
}

int	test_setNextVertex(Vertex* v1, Vertex* v2)
{
	v1->setNextVertex(v2);
	
	assert(v1->getNextVertex() == v2);
	
	return 0;
}

int main()
{
	Vertex* v1 = new Vertex(5, 3);
	Vertex* v2 = new Vertex(10, 9);
	
	test_Vertex(v1);
	test_getPoints(v1);
	test_setPoints(v1);
	test_getVisited(v1);
	test_setVisited(v1);
	test_getNextVertex(v1);
	test_setNextVertex(v1, v2);
}




















