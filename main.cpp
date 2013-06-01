// PruebaMRcore.cpp : Defines the entry point for the console application.
//


#include "mrcore.h"
#include <iostream>


using namespace mr;
using namespace std;

void main()
{
	//////////////////////////// WRITE ////////////////////////////////////////

///************** PRUEBA DE ESFERA  PARA WRITETOXML **********////////////////
	SpherePart sphere;
	sphere.setName("pruebaSphere");
	XML* xml = new XML("myXML.xml");
	XMLElement* r = xml->GetRootElement();
	sphere.writeToXML(r);
	xml->Save();

////*************** PRUEBA DE PRISMA PARA WRITETOXML **********/////////

	//PrismaticPart prism;
	//vector<Vector2D> vertex;
	//XMLAux c;
	//

	//for (int i=0;i<5;i++)
	//vertex.push_back(Vector2D(i+0.2,i-0.2));

	//prism.setPolygonalBase(vertex);
	//prism.setName("prismaPrueba");
	//XML* xml1 = new XML("myXML1.xml");
	//XMLElement* r1 = xml1->GetRootElement();
	//XMLElement* tree=new XMLElement(r1,"árbol");
	//prism.writeToXML(r1);

	//xml1->Save();
///********************************************************************//////////////////////


	//////////////////////////////////// READ //////////////////////////////////////
//********************** PRUEBA PARA LECTURA READFROMXML DE ESFERA  ***************////////////////////


	//SpherePart sphere;
	//XML* xml = new XML("myXML.xml");
	//XMLElement* r = xml->GetRootElement();
	////sphere.readFromXML(r);
	//sphere.writeToXML(r);
	//xml->Save();

//********************** PRUEBA PARA LECTURA READFROMXML DE PRISMA **************/////////////////////////
	//PrismaticPart prism;
	//XML* xml1 = new XML("myXML1.xml");
	//XMLElement* r1 = xml1->GetRootElement();
	//XMLElement* tree=new XMLElement(r1,"árbol");
	//prism.readFromXML(r1);
	//prism.writeToXML(r1);
	//xml1->Save();

///**********************************************************************************************************///////


}