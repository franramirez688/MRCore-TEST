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
	
	//SpherePart* sphere=new SpherePart;
	////sphere->setName("Sphereere");
	//sphere->setColor(0.561,0.248,0.2544);
	//XMLfile file;
	//file<<sphere;
	//file.save();


////*************** PRUEBA DE PRISMA PARA WRITETOXML **********/////////

	//PrismaticPart *prism=new PrismaticPart();
	//vector<Vector2D> vertex;

	//for (int i=0;i<5;i++)
	//vertex.push_back(Vector2D(i+0.2,i-0.2));

	//prism->setPolygonalBase(vertex);
	//prism->setName("prismaPrueba");

	//XMLfile file;
	//file<<prism
	//file.save();

///********************************************************************//////////////////////


	//////////////////////////////////// READ //////////////////////////////////////
//********************** PRUEBA PARA LECTURA READFROMXML DE ESFERA  ***************////////////////////


	//SpherePart* sphere=new SpherePart();
	////XML* xml = new XML("xml_file.xml");
	////XMLElement* r = xml->GetRootElement();
	////sphere->readFromXML(r);
	////sphere->writeToXML(r);
	////xml->Save();

	//XMLfile file("xml_file.xml");
	//sphere=dynamic_cast <SpherePart*> (file.load("xml_file.xml"));
	//file.write(sphere);
	//file.save();


//********************** PRUEBA PARA LECTURA READFROMXML DE PRISMA **************/////////////////////////
	//PrismaticPart *prism=new PrismaticPart();
	//XML* xml1 = new XML("myXML1.xml");
	//XMLElement* r1 = xml1->GetRootElement();
	//XMLElement* tree=new XMLElement(r1,"árbol");
	//prism.readFromXML(r1);
	//prism.writeToXML(r1);
	//xml1->Save();

///**********************************************************************************************************///////


}