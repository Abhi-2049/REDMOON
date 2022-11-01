//
// ********************************************************************
// * License and Disclaimer                                           *
// *                                                                  *
// * The  Geant4 software  is  copyright of the Copyright Holders  of *
// * the Geant4 Collaboration.  It is provided  under  the terms  and *
// * conditions of the Geant4 Software License,  included in the file *
// * LICENSE and available at  http://cern.ch/geant4/license .  These *
// * include a list of copyright holders.                             *
// *                                                                  *
// * Neither the authors of this software system, nor their employing *
// * institutes,nor the agencies providing financial support for this *
// * work  make  any representation or  warranty, express or implied, *
// * regarding  this  software system or assume any liability for its *
// * use.  Please see the license in the file  LICENSE  and URL above *
// * for the full disclaimer and the limitation of liability.         *
// *                                                                  *
// * This  code  implementation is the result of  the  scientific and *
// * technical work of the GEANT4 collaboration.                      *
// * By using,  copying,  modifying or  distributing the software (or *
// * any work based  on the software)  you  agree  to acknowledge its *
// * use  in  resulting  scientific  publications,  and indicate your *
// * acceptance of all terms of the Geant4 Software license.          *
// ********************************************************************
//
//
/// \file DetectorConstruction.cc
/// \brief Implementation of the B1::DetectorConstruction class

/*
// Test_4: Simplified Payload MAIN with rover chassis, 200 MeV proton beam of 10 cm X 10 cm square field

Shape 1 - Main PCB
Shape 2 - Main FGDOS

Shape 3 - Al Chassis block
Shape 4 - Internal cavity

Shape 5 - SPP

Shape 6 - Daughterboard PCB
Shape 7 - Daughter FGDOS

*/


#include "DetectorConstruction.hh"

#include "G4RunManager.hh"
#include "G4NistManager.hh"
#include "G4Box.hh"
#include "G4Cons.hh"
#include "G4Orb.hh"
#include "G4Sphere.hh"
#include "G4Trd.hh"
#include "G4LogicalVolume.hh"
#include "G4PVPlacement.hh"
#include "G4SystemOfUnits.hh"

#include "G4SubtractionSolid.hh"																											// $$

namespace B1
{

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

DetectorConstruction::DetectorConstruction()
{}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

DetectorConstruction::~DetectorConstruction()
{}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

G4VPhysicalVolume* DetectorConstruction::Construct()
{
  // Get nist material manager
  G4NistManager* nist = G4NistManager::Instance();

  // Envelope parameters
  //
  G4double env_sizeXY = 200*cm, env_sizeZ = 200*cm;												// $$  G4double env_sizeXY = 20*cm, env_sizeZ =30*cm;
  G4Material* env_mat = nist->FindOrBuildMaterial("G4_Galactic");  				// $$  G4_WATER

  // Option to switch on/off checking of volumes overlaps
  //
  G4bool checkOverlaps = true;

  //
  // World
  //
  G4double world_sizeXY = 1.2*env_sizeXY;
  G4double world_sizeZ  = 1.2*env_sizeZ;
  G4Material* world_mat = nist->FindOrBuildMaterial("G4_Galactic");

  G4Box* solidWorld =
    new G4Box("World",                       //its name
       0.5*world_sizeXY, 0.5*world_sizeXY, 0.5*world_sizeZ);     //its size

  G4LogicalVolume* logicWorld =
    new G4LogicalVolume(solidWorld,          //its solid
                        world_mat,           //its material
                        "World");            //its name

  G4VPhysicalVolume* physWorld =
    new G4PVPlacement(0,                     //no rotation
                      G4ThreeVector(),       //at (0,0,0)
                      logicWorld,            //its logical volume
                      "World",               //its name
                      0,                     //its mother  volume
                      false,                 //no boolean operation
                      0,                     //copy number
                      checkOverlaps);        //overlaps checking

  //
  // Envelope
  //
  G4Box* solidEnv =
    new G4Box("Envelope",                    //its name
        0.5*env_sizeXY, 0.5*env_sizeXY, 0.5*env_sizeZ); //its size

  G4LogicalVolume* logicEnv =
    new G4LogicalVolume(solidEnv,            //its solid
                        env_mat,             //its material
                        "Envelope");         //its name

  new G4PVPlacement(0,                       //no rotation
                    G4ThreeVector(),         //at (0,0,0)
                    logicEnv,                //its logical volume
                    "Envelope",              //its name
                    logicWorld,              //its mother  volume
                    false,                   //no boolean operation
                    0,                       //copy number
                    checkOverlaps);          //overlaps checking



  // Material definition for FR4
  
  //Epoxy (for FR4 ) 
  //density = 1.2*g/cm3; 
  G4Material* Epoxy = new G4Material("Epoxy", 1.2*g/cm3, 2); 
  Epoxy->AddElement(nist->FindOrBuildElement("H"), 2); 
	Epoxy->AddElement(nist->FindOrBuildElement("C"), 2); 

	//SiO2 (for FR4 ) 
	//density = 2.2*g/cm3; 
	G4Material* SiO2 =  
	new G4Material("SiO2",2.2*g/cm3, 2); 
	SiO2->AddElement(nist->FindOrBuildElement("Si"), 1); 
	SiO2->AddElement(nist->FindOrBuildElement("O"), 2); 

	//FR4 (Glass + Epoxy) 
	//density = 1.86*g/cm3; 
	G4Material* FR4 = new G4Material("FR4", 1.86*g/cm3, 2); 
	FR4->AddMaterial(SiO2, 52.8*perCent); 
	FR4->AddMaterial(Epoxy, 47.2*perCent); 



  //
  // Shape 1 - Radiation Payload MAIN PCB - FR4 - Box - 4.2 cm X 4.2 cm X 0.16 cm
  //
  G4Material* shape1_mat = nist->FindOrBuildMaterial("FR4");
  G4ThreeVector pos1 = G4ThreeVector(0, 0*cm, -0.08*cm);

  // Box shape
	G4double shape1_hx =  4.2*cm;
  G4double shape1_hy =  4.2*cm;
  G4double shape1_hz = 0.16*cm;  
	
	G4Box* solidShape1 =
    new G4Box("Shape1",                    //its name
        0.5*shape1_hx, 0.5*shape1_hy, 0.5*shape1_hz); //its size

	// Logical volume - shape 1
  G4LogicalVolume* logicShape1 =
    new G4LogicalVolume(solidShape1,         //its solid
                        shape1_mat,          //its material
                        "Shape1");           //its name
	// Physical volume - shape 1
  new G4PVPlacement(0,                       //no rotation
                    pos1,                    //at position
                    logicShape1,             //its logical volume
                    "Shape1",                //its name
                    logicEnv,                //its mother  volume
                    false,                   //no boolean operation
                    0,                       //copy number
                    checkOverlaps);          //overlaps checking

  
	//
  // Shape 2 - FGDOS Detector - Si - Box - 0.5 cm X 0.5 cm X 0.1 cm
  //
  G4Material* shape2_mat = nist->FindOrBuildMaterial("G4_Si");
  G4ThreeVector pos2 = G4ThreeVector(0, 0*cm, 0.05*cm);

  // Box shape
	G4double shape2_hx =  0.5*cm;
  G4double shape2_hy =  0.5*cm;
  G4double shape2_hz =  0.1*cm;

  G4Box* solidShape2 =
    new G4Box("Shape2",                    //its name
        0.5*shape2_hx, 0.5*shape2_hy, 0.5*shape2_hz); //its size
	
	// Logical volume - shape 2
  G4LogicalVolume* logicShape2 =
    new G4LogicalVolume(solidShape2,         //its solid
                        shape2_mat,          //its material
                        "Shape2");           //its name

	// Physical volume - shape 2
  new G4PVPlacement(0,                       //no rotation
                    pos2,                    //at position
                    logicShape2,             //its logical volume
                    "Shape2",                //its name
                    logicEnv,                //its mother  volume
                    false,                   //no boolean operation
                    0,                       //copy number
                    checkOverlaps);          //overlaps checking

//////////////////////////////////////////////////////////////////////////////////////////////////

  //
  // Shape 3 - Rover Chassis Block - Al - Box - 20 cm X 14 cm X 6.65 cm
  //
  G4Material* shape3_mat = nist->FindOrBuildMaterial("G4_Al");
  G4ThreeVector pos3 = G4ThreeVector(-6.3*cm, 0*cm, -1.88*cm);									// $$ (0, 0*cm, -0.08*cm)

  // Box shape
	G4double shape3_hx =  20.0*cm;
  G4double shape3_hy =  14.0*cm;
  G4double shape3_hz = 	6.65*cm;  
	
	G4Box* solidShape3 =
    new G4Box("Shape3",                    //its name
        0.5*shape3_hx, 0.5*shape3_hy, 0.5*shape3_hz); //its size
/*
	// Logical volume - shape 3
  G4LogicalVolume* logicShape3 =
    new G4LogicalVolume(solidShape3,         //its solid
                        shape3_mat,          //its material
                        "Shape3");           //its name
	// Physical volume - shape 3
  new G4PVPlacement(0,                       //no rotation
                    pos3,                    //at position
                    logicShape3,             //its logical volume
                    "Shape3",                //its name
                    logicEnv,                //its mother  volume
                    false,                   //no boolean operation
                    0,                       //copy number
                    checkOverlaps);          //overlaps checking

*/


  //
  // Shape 4 - Rover Chassis Cavity - Negative volume - Box - 19.7 cm X 13.7 cm X 6.35 cm
  //
  G4Material* shape4_mat = nist->FindOrBuildMaterial("G4_Galactic");
  G4ThreeVector pos4 = G4ThreeVector(-6.3*cm, 0*cm, -1.88*cm);									// $$ (0, 0*cm, -0.08*cm)

  // Box shape
	G4double shape4_hx =  19.7*cm;
  G4double shape4_hy =  13.7*cm;
  G4double shape4_hz = 	6.35*cm;  
	
	G4Box* solidShape4 =
    new G4Box("Shape4",                    //its name
        0.5*shape4_hx, 0.5*shape4_hy, 0.5*shape4_hz); //its size


	


	// Shape 8 - Solid formed by Boolean subtraction of shape 4 from shape 3

	G4SubtractionSolid* solidShape8 = 
  new G4SubtractionSolid("Shape8", solidShape3, solidShape4); 


	// Logical volume - shape 8 - Rover chassis only
  G4LogicalVolume* logicShape8 =
    new G4LogicalVolume(solidShape8,         //its solid
                        shape3_mat,          //its material
                        "Shape8");           //its name
	// Physical volume - shape 8 - Rover chassis only
  new G4PVPlacement(0,                       //no rotation
                    pos3,                    //at position
                    logicShape8,             //its logical volume
                    "Shape8",                //its name
                    logicEnv,                //its mother  volume
                    false,                   //no boolean operation
                    0,                       //copy number
                    checkOverlaps);          //overlaps checking





  // Material definition for SiC (Ceramic Matrix Composite)
  
	//SiC
	//density = 1.9-2.0*g/cm3; 
	G4Material* SiC =  
	new G4Material("SiC",2.0*g/cm3, 2); 
	SiC->AddElement(nist->FindOrBuildElement("Si"), 1); 
	SiC->AddElement(nist->FindOrBuildElement("C"), 1); 

  //
  // Shape 5 - Solar Panel Plate - CMC - Box - 0.2 cm X 17.5 cm X 33.5 cm
  //
  G4Material* shape5_mat = nist->FindOrBuildMaterial("SiC");
  G4ThreeVector pos5 = G4ThreeVector(-16.4*cm, 0*cm, 18.2*cm);										// $$ (0, 0*cm, -0.08*cm)

  // Box shape
	G4double shape5_hx =  0.2*cm;
  G4double shape5_hy =  17.5*cm;
  G4double shape5_hz = 	33.5*cm;  
	
	G4Box* solidShape5 =
    new G4Box("Shape5",                    //its name
        0.5*shape5_hx, 0.5*shape5_hy, 0.5*shape5_hz); //its size

	// Logical volume - shape 5
  G4LogicalVolume* logicShape5 =
    new G4LogicalVolume(solidShape5,         //its solid
                        shape5_mat,          //its material
                        "Shape5");           //its name
	// Physical volume - shape 5
  new G4PVPlacement(0,                       //no rotation
                    pos5,                    //at position
                    logicShape5,             //its logical volume
                    "Shape5",                //its name
                    logicEnv,                //its mother  volume
                    false,                   //no boolean operation
                    0,                       //copy number
                    checkOverlaps);          //overlaps checking



////////////////////////////////////////////////////////////////////////////////////////////////

  //
  // Shape 6 - Radiation Payload Daughterboard PCB - FR4 - Box - 4.2 cm X 4.2 cm X 0.16 cm
  //
  //G4Material* shape6_mat = nist->FindOrBuildMaterial("FR4");
  G4ThreeVector pos6 = G4ThreeVector(-16.2*cm, 0*cm, 18.2*cm);												// $$ (0, 0*cm, -0.08*cm)

  // Box shape
	G4double shape6_hx =  0.16*cm;
  G4double shape6_hy =  4.2*cm;
  G4double shape6_hz = 	4.2*cm;  
	
	G4Box* solidShape6 =
    new G4Box("Shape6",                    //its name
        0.5*shape6_hx, 0.5*shape6_hy, 0.5*shape6_hz); //its size

	// Logical volume - shape 6
  G4LogicalVolume* logicShape6 =
    new G4LogicalVolume(solidShape6,         //its solid
                        shape1_mat,          //its material
                        "Shape6");           //its name
	// Physical volume - shape 6
  new G4PVPlacement(0,                       //no rotation
                    pos6,                    //at position
                    logicShape6,             //its logical volume
                    "Shape6",                //its name
                    logicEnv,                //its mother  volume
                    false,                   //no boolean operation
                    0,                       //copy number
                    checkOverlaps);          //overlaps checking

  
	//
  // Shape 7 - Daughterboard FGDOS Detector - Si - Box - 0.5 cm X 0.5 cm X 0.1 cm
  //
  //G4Material* shape7_mat = nist->FindOrBuildMaterial("G4_Si");
  G4ThreeVector pos7 = G4ThreeVector(-16.07*cm, 0*cm, 18.2*cm);												// $$ (0, 0*cm, 0.05*cm)

  // Box shape
	G4double shape7_hx =  0.1*cm;
  G4double shape7_hy =  0.5*cm;
  G4double shape7_hz =  0.5*cm;

  G4Box* solidShape7 =
    new G4Box("Shape7",                    //its name
        0.5*shape7_hx, 0.5*shape7_hy, 0.5*shape7_hz); //its size
	
	// Logical volume - shape 7
  G4LogicalVolume* logicShape7 =
    new G4LogicalVolume(solidShape7,         //its solid
                        shape2_mat,          //its material
                        "Shape7");           //its name

	// Physical volume - shape 7
  new G4PVPlacement(0,                       //no rotation
                    pos7,                    //at position
                    logicShape7,             //its logical volume
                    "Shape7",                //its name
                    logicEnv,                //its mother  volume
                    false,                   //no boolean operation
                    0,                       //copy number
                    checkOverlaps);          //overlaps checking





//////////////////////////////////////////////////////////////////////////////////////////////////




  // Set Shape2 - MAIN FGDOS as scoring volume
  //
  fScoringVolume = logicShape2;


///////////////////////////////////////////////////////////////////////////////////////////////////

	// Set Shape7 - Daughter FGDOS as another scoring volume
	//
	//fScoringVolume = logicShape7;
 




///////////////////////////////////////////////////////////////////////////////////////////////////

  //
  //always return the physical World
  //
  return physWorld;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

}
