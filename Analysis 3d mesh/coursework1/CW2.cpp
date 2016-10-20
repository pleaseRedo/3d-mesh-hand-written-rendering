#include <stdlib.h>
#include <stdio.h>
#include <OpenGL/gl.h> 
#include <OpenGL/glu.h>
#include <GLUT/GLUT.h>
#include <math.h>
#include "tgaload.h"
#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "objloader.h"
#include "OpenGLSupport"
char rendermode; // global variable for current rendering mode
//Global variables

//Hold coordinates of all vertices
float vertex[8][3] ={{1,1,1},{-1,1,1},{-1,-1,1},{1,-1,1},{1,-1,-1},{-1,-1,-1},{1,1,-1},{-1,1,-1}};
// Numbered the faces of cube : FRONT 1 RIGHT 2 BACK 3 LEFT 4 TOP 5 BOTTOM 6
//Nearby faces of each vertex
int faces[8][3] ={{0,1,4},{0,3,4},{0,3,5},{0,1,5},{1,2,5},{2,3,5},{1,2,4},{2,3,4}};
//Vertices for each face
int verticesForFace[6][4] = {{0,1,2,3},{6,0,3,4},{6,7,5,4},{7,1,2,5},{6,7,1,0},{4,5,2,3}};
int edgesMap[6][8]={{1,2,2,3,3,0,0,1},
                    {0,3,3,4,4,6,6,0},
                    {7,5,5,4,4,6,6,7},
                    {1,2,2,5,5,7,7,1},    //Stores the vertices' number of each edges on each faces
                    {1,0,0,6,6,7,7,1},
                    {2,3,3,4,4,5,5,2}};
float faceNormal[6][3];// Stores the face normal of each face
float textureMap[4][2]={{0,0},{0,1},{1,1},{1,0}};

//Camera rotation angle
double theta = 0;
double alpha = 30;
double radius = 5; // camera rotation radius

//glulookat eye and  target coordinate
double eyez =3;
double eyex =0;
double eyey =0;
double posx=0;
double posy= 0;
double posz = 0;
double rotateX=0;
double rotateY=0;
double rotateZ=0;

GLuint g_textureID[1];

float fov = 45;//Field of View
//Vectors of glulookat.
float cameraFront[3] = {0.0,0.0,3.0};
float cameraPos[3] = {0,0,-1};
float cameraUp[3] = {0,1,0};


bool cameraRotation; // Whether camera rotation were enabled
bool mousePress; // Whether mouse has been clicked.


// Vectors for CW2
std::vector<Eigen::Vector3d> vertices;
std::vector<Eigen::Vector3d> faceIndices;

Eigen::Matrix3f covarianceM;  //Covariance Matrix
Eigen::MatrixXcf eivects;     //Eigen vector
Eigen::MatrixXf eiVector;     //Eigen vector in real number
Eigen::MatrixXf eiVectorInverse;// The inverse matrix of eiVector

bool showOBB; //Dis/En OBB drawing;
bool showABB; //DIS/En ABB drawing;
//Maximum and minimum value of coordinate(x,y,z) in mesh.
float maxX ;
float maxY ;
float maxZ ;
float minX ;
float minY ;
float minZ ;
//The coordinates of the centre of mesh.
float meshCentreX;
float meshCentreY;
float meshCentreZ;
// Scale vector in xyz, and the maximum value amoung them scaleVector.
float scaleX;
float scaleY;
float scaleZ;
float scaleVector;
// Mesh rotation value.
float meshAngleX = 0.1f;
float meshAngleY = 0.1f;
float meshAngleZ = 0.1f;
// Mean of mesh in xyz coordinate.
float meshXMean;
float meshYMean;
float mesjZMean;
//convariance value
float covXX, covXY, covXZ, covYY, covYZ, covZZ;
// The max and min value for the p' in object reference frame
float obbMaxX,obbMaxY,obbMaxZ, obbMinX,obbMinY,obbMinZ;
//Vertices of OBB.
float obbPoints[8][3] = {{obbMaxX,obbMaxY,obbMaxZ},
                           {obbMinX,obbMaxY,obbMaxZ},
                           {obbMinX,obbMinY,obbMaxZ},
                           {obbMaxX,obbMinY,obbMaxZ},
                           {obbMaxX,obbMinY,obbMinZ},
                           {obbMinX,obbMinY,obbMinZ},
                           {obbMaxX,obbMaxY,obbMinZ},
                           {obbMinX,obbMaxY,obbMinZ}};
//RotationX matrix.
float Rx[4][4] = {{1,0,0,0},
                 {0, cosf(-meshAngleX),-sinf(-meshAngleX),0},
                 {0,sinf(-meshAngleX),cosf(-meshAngleX),0},
                 {0,0,0,1}};
//RotationY matrix
float Ry[4][4] = {{cosf(-meshAngleY),0,sinf(-meshAngleY),0},
                 {0,1,0,0},
                 {-sinf(-meshAngleY),0,cosf(-meshAngleY),0},
                 {0,0,0,1}};
//RotationZ matrix
float Rz[4][4] = {{cosf(-meshAngleZ),-sinf(-meshAngleZ),0,0},
                 {sinf(-meshAngleZ),cosf(-meshAngleZ),0,0},
                 {0,0,1,0},
                 {0,0,0,1}};


/*
 Rotate the mesh along X
 */
void rotateMeshX(){
    //Applying Rx matrix to every points in mesh vertices.
    for (int i=0; i<vertices.size(); i++) {
        float x = vertices[i][0];
        float y = vertices[i][1];
        float z = vertices[i][2];
        vertices[i][0] = (Rx[0][0] * x) + (Rx[0][1] * y) + (Rx[0][2] * z) + Rx[0][3];
        vertices[i][1] = (Rx[1][0] * x) + (Rx[1][1] * y) + (Rx[1][2] * z) + Rx[1][3];
        vertices[i][2] = (Rx[2][0] * x) + (Rx[2][1] * y) + (Rx[2][2] * z) + Rx[2][3];
    }
    //Applying Rx matrix to every points in obb vertices.
    for (int i=0; i<8 ; i++) {
        float x = obbPoints[i][0];
        float y = obbPoints[i][1];
        float z = obbPoints[i][2];
        obbPoints[i][0] = (Rx[0][0] * x) + (Rx[0][1] * y) + (Rx[0][2] * z) + Rx[0][3];
        obbPoints[i][1] = (Rx[1][0] * x) + (Rx[1][1] * y) + (Rx[1][2] * z) + Rx[1][3];
        obbPoints[i][2] = (Rx[2][0] * x) + (Rx[2][1] * y) + (Rx[2][2] * z) + Rx[2][3];

    }

}
/*
 Rotate the mesh along Y
 */
void rotateMeshY(){
    //Applying Ry matrix to every points in mesh vertices.
    for (int i=0; i<vertices.size(); i++) {
        float x = vertices[i][0];
        float y = vertices[i][1];
        float z = vertices[i][2];
        vertices[i][0] = (Ry[0][0] * x)+(Ry[0][1] * y)+(Ry[0][2] * z) + Ry[0][3];
        vertices[i][1] = (Ry[1][0] * x)+(Ry[1][1] * y)+(Ry[1][2] * z) + Ry[1][3];
        vertices[i][2] = (Ry[2][0] * x)+(Ry[2][1] * y)+(Ry[2][2] * z) + Ry[2][3];

    }
    //Applying Ry matrix to every points in obb vertices.

    for (int i=0; i<8 ; i++) {
        float x = obbPoints[i][0];
        float y = obbPoints[i][1];
        float z = obbPoints[i][2];
        obbPoints[i][0] = (Ry[0][0] * x) + (Ry[0][1] * y) + (Ry[0][2] * z) + Ry[0][3];
        obbPoints[i][1] = (Ry[1][0] * x) + (Ry[1][1] * y) + (Ry[1][2] * z) + Ry[1][3];
        obbPoints[i][2] = (Ry[2][0] * x) + (Ry[2][1] * y) + (Ry[2][2] * z) + Ry[2][3];
        
    }
}
/*
 Rotate the mesh along Z
 */
void rotateMeshZ(){
    //Applying Rz matrix to every points in mesh vertices.
    for (int i=0; i<vertices.size(); i++) {
        float x = vertices[i][0];
        float y = vertices[i][1];
        float z = vertices[i][2];
        vertices[i][0] = (Rz[0][0] * x)+(Rz[0][1] * y)+(Rz[0][2] * z) + Rz[0][3];
        vertices[i][1] = (Rz[1][0] * x)+(Rz[1][1] * y)+(Rz[1][2] * z) + Rz[1][3];
        vertices[i][2] = (Rz[2][0] * x)+(Rz[2][1] * y)+(Rz[2][2] * z) + Rz[2][3];
    }
    //Applying Rz matrix to every points in obb vertices.
    for (int i=0; i<8 ; i++) {
        float x = obbPoints[i][0];
        float y = obbPoints[i][1];
        float z = obbPoints[i][2];
        obbPoints[i][0] = (Rz[0][0] * x) + (Rz[0][1] * y) + (Rz[0][2] * z) + Rz[0][3];
        obbPoints[i][1] = (Rz[1][0] * x) + (Rz[1][1] * y) + (Rz[1][2] * z) + Rz[1][3];
        obbPoints[i][2] = (Rz[2][0] * x) + (Rz[2][1] * y) + (Rz[2][2] * z) + Rz[2][3];
        
    }
}




void setuptexture() {
    unsigned char* image = NULL;
    int iheight, iwidth;
    image_t tex_image; //only needed for tga
    

    // Load image from file
    tgaLoad("brick.tga", &tex_image, TGA_FREE | TGA_LOW_QUALITY);

    
    // Create a texture object with a unused texture ID
    glGenTextures(1, g_textureID);
    
    // Set g_textureID as the current 2D texture object
    glBindTexture(GL_TEXTURE_2D, g_textureID[0]);
    
    // Set the loaded image as the current texture image
    glTexImage2D ( GL_TEXTURE_2D, 0, tex_image.info.tgaColourType,
    tex_image.info.width, tex_image.info.height, 0,
    tex_image.info.tgaColourType, GL_UNSIGNED_BYTE, tex_image.data );
 
    
    // Specify what to do when s, t outside range [0, 1]
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    
    // Specify how to interpolate texture color values
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}


/*
 Do the vector addition
 */
float * vectorAddition(float vector1[3],float vector2[3]){
    static float v[3];
    float element1;
    float element2;
    float element3;
    
    element1 = vector1[0] + vector2[0];
    element2 = vector1[1] + vector2[1];
    element3 = vector1[2] + vector2[2];
    v[0] = element1;
    v[1] = element2;
    v[2] = element3;
    
    return v;
}
/*
 Callucate the cross product of 2 vectors and normalised them
 */
float * calCrossProduct(float u[3], float v[3]){
    static float c[3];
    float element1;
    float element2;
    float element3;
    float magnitude;
    element1 = (u[1]*v[2]-u[2]*v[1]);
    element2 = (u[2]*v[0]-u[0]*v[2]);
    element3 = (u[0]*v[1]-u[1]*v[0]);
    magnitude = sqrtf(powf(element1, 2)+powf(element2, 2)+powf(element3, 2));
    //Normalize
    c[0] = element1/magnitude;
    c[1] = element2/magnitude;
    c[2] = element3/magnitude;

    return c;
}
/*
 Get the normal for each face
 */
void assignNormal (float pointA[3],float pointB[3],float pointC[3],int face){
    float AB[3];
    float AC[3];
    float *temp;
    AB[0] = pointB[0] - pointA[0];
    AB[1] = pointB[1] - pointA[1];
    AB[2] = pointB[2] - pointA[2];
    
    
    AC[0] = pointC[0] - pointA[0];
    AC[1] = pointC[1] - pointA[1];
    AC[2] = pointC[2] - pointA[2];
    
    temp = calCrossProduct(AB, AC);
    faceNormal[face][0] = temp[0];
    faceNormal[face][1] = temp[1];
    faceNormal[face][2] = temp[2];

}
/*
 Calculating the average value of input 3 normals.
 */
float * calNormalAverage(int face[3]){
    
    static float average[3];
    average[0] = (faceNormal[face[0]][0] + faceNormal[face[1]][0] + faceNormal[face[2]][0])/3;
    average[1] = (faceNormal[face[0]][1] + faceNormal[face[1]][1] + faceNormal[face[2]][1])/3;
    average[2] = (faceNormal[face[0]][2] + faceNormal[face[1]][2] + faceNormal[face[2]][2])/3;

    return average;
}

/*
 Store the normal value to an array.
 */
void updateNormal(){
    assignNormal(vertex[0], vertex[1], vertex[3], 0);
    assignNormal(vertex[0], vertex[3], vertex[6], 1);
    assignNormal(vertex[6], vertex[4], vertex[7], 2);
    assignNormal(vertex[7], vertex[5], vertex[1], 3);
    assignNormal(vertex[0], vertex[6], vertex[1], 4);
    assignNormal(vertex[3], vertex[2], vertex[4], 5);

}

/*
 Drawing cube and assign normal to each vertex.
 */
void drawCube(void)
{
    float *average;
    updateNormal();
    glEnable(GL_NORMALIZE);
    
    for (int i=0; i<6; i++) {
        glBegin(GL_POLYGON);
        glColor3f(1.0f,0.0f,0.0f);
        for (int j =0; j<4; j++) {
            average = calNormalAverage(faces[verticesForFace[i][j]]);
            glNormal3fv(average);
            glTexCoord2fv(textureMap[j]);
            glVertex3fv(vertex[verticesForFace[i][j]]);
        }
        glEnd();
    }

}

/*
 Calculate the mean of specified coordinates
 */

float calMean(int index){ // 0 for x, 1 for y , 2 for z
    float mean = 0;
    for (int i=0; i<vertices.size(); i++) {
        mean += vertices[i][index];
    }
    mean = mean/vertices.size();
    return mean;
}
/*
 Calculate the covariance of vertices in mesh
 */
void calCovar(){
    for (int i ; i<vertices.size(); i++) {
        covXX += (vertices[i][0]-calMean(0))*(vertices[i][0]-calMean(0));
        covXY += (vertices[i][0]-calMean(0))*(vertices[i][1]-calMean(1));
        covXZ += (vertices[i][0]-calMean(0))*(vertices[i][2]-calMean(2));
        covYY += (vertices[i][1]-calMean(1))*(vertices[i][1]-calMean(2));
        covYZ += (vertices[i][1]-calMean(1))*(vertices[i][2]-calMean(2));
        covZZ += (vertices[i][2]-calMean(2))*(vertices[i][2]-calMean(2));
        
    }
    covXX = covXX/vertices.size();
    covXY = covXY/vertices.size();
    covXZ = covXZ/vertices.size();
    covYY = covYY/vertices.size();
    covYZ = covYZ/vertices.size();
    covZZ = covZZ/vertices.size();
}
/*
 Update the covariance value to the Covariance Matrix covarianceM.
 */
void updateCovar(){
    
    calCovar();                 // Calculate the value first,
    covarianceM(0,0) = covXX;   // then assign value to each places in covarianceM.
    covarianceM(0,1) = covXY;
    covarianceM(0,2) = covXZ;
    covarianceM(1,0) = covXY;
    covarianceM(1,1) = covYY;
    covarianceM(1,2) = covYZ;
    covarianceM(2,0) = covXZ;
    covarianceM(2,1) = covYZ;
    covarianceM(2,2) = covZZ;
    
    
    
    //Create EigenSolver es to get eigen vector and value.
    Eigen::EigenSolver<Eigen::MatrixXf> es(covarianceM);
    eivects=es.eigenvectors();
    eiVector=eivects.real();  // Get eivector in real number.
}

/*
 Calculate the max and min of p' in object reference frame.
 */
void calOBBMM(){
    
    
    obbMaxX = 0;obbMaxY = 0;obbMaxZ = 0;obbMinX = 0;obbMinY = 0;obbMinZ = 0;
    float tempx = 0 ,tempy = 0,tempz=0;
    float meanx = calMean(0);float meany=calMean(1);float meanz=calMean(2);
    for (int i=0; i<vertices.size(); i++) {
        // Applying inverse matrix of eiVector to each point by doing multiplication and subtract the mean every time to work out p' for
        // each points.
        
        tempx = eiVector.transpose()(0,0)*(vertices[i][0]-meanx) + eiVector.transpose()(0,1)*(vertices[i][1]-meany)+ eiVector.transpose()(0,2)*(vertices[i][2]-meanz);
        if (obbMaxX < tempx ) {     //Doing comparison each time to handle the current maximun and minimum value.

            obbMaxX = tempx;
        }
        if (obbMinX > tempx ) {
            obbMinX = tempx;
        }
        
        
        
        
        tempy = eiVector.transpose()(1,0)*(vertices[i][0]-meanx) + eiVector.transpose()(1,1)*(vertices[i][1]-meany)+ eiVector.transpose()(1,2)*(vertices[i][2]-meanz);
        if (obbMaxY < tempy ) {
            obbMaxY = tempy;
        }
        if (obbMinY > tempy ) {
            obbMinY = tempy;
        }
        tempz = eiVector.transpose()(2,0)*(vertices[i][0]-meanx) + eiVector.transpose()(2,1)*(vertices[i][1]-meany)+ eiVector.transpose()(2,2)*(vertices[i][2]-meanz);
        
        if (obbMaxZ < tempz ) {
            obbMaxZ = tempz;
        }
        if (obbMinZ > tempz ) {
            obbMinZ = tempz;
        }
    }

    // Update the value to obbPoints matrix.
    obbPoints[0][0] = obbMaxX;
    obbPoints[0][1] = obbMaxY;
    obbPoints[0][2] = obbMaxZ;

    obbPoints[1][0] = obbMinX;
    obbPoints[1][1] = obbMaxY;
    obbPoints[1][2] = obbMaxZ;
    
    obbPoints[2][0] = obbMinX;
    obbPoints[2][1] = obbMinY;
    obbPoints[2][2] = obbMaxZ;
    
    obbPoints[3][0] = obbMaxX;
    obbPoints[3][1] = obbMinY;
    obbPoints[3][2] = obbMaxZ;
    
    obbPoints[4][0] = obbMaxX;
    obbPoints[4][1] = obbMinY;
    obbPoints[4][2] = obbMinZ;
    
    obbPoints[5][0] = obbMinX;
    obbPoints[5][1] = obbMinY;
    obbPoints[5][2] = obbMinZ;
    
    obbPoints[6][0] = obbMaxX;
    obbPoints[6][1] = obbMaxY;
    obbPoints[6][2] = obbMinZ;
    
    obbPoints[7][0] = obbMinX;
    obbPoints[7][1] = obbMaxY;
    obbPoints[7][2] = obbMinZ;
    
    
}
/*
 Return max and min value of vertices in mesh.
 */
float meshMaxX()
{
    float maxValue = 0;
    for ( unsigned int i=0; i<vertices.size(); i++ ){
        if(vertices[i][0] >maxValue){
            maxValue = vertices[i][0];
        }
    }
    return maxValue;
}
float meshMaxY()
{
    float maxValue = 0;
    for ( unsigned int i=0; i<vertices.size(); i++ ){
        if(vertices[i][1] >maxValue){
            maxValue = vertices[i][1];
        }
    }
    return maxValue;
}
float meshMaxZ()
{
    float maxValue = 0;
    for ( unsigned int i=0; i<vertices.size(); i++ ){
        if(vertices[i][2] >maxValue){
            maxValue = vertices[i][2];
        }
    }
    return maxValue;
}
float meshMinX()
{
    float minValue = 0;
    for ( unsigned int i=0; i<vertices.size(); i++ ){
        if(vertices[i][0] < minValue){
            minValue = vertices[i][0];
        }
    }
    return minValue;
}
float meshMinY()
{
    float minValue = 0;
    for ( unsigned int i=0; i<vertices.size(); i++ ){
        if(vertices[i][1] < minValue){
            minValue = vertices[i][1];
        }
    }
    return minValue;
}
float meshMinZ()
{
    float minValue = 0;
    for ( unsigned int i=0; i<vertices.size(); i++ ){
        if(vertices[i][2] < minValue){
            minValue = vertices[i][2];
        }
    }
    return minValue;
}
/*
 Calculate the max ,min and scale vector of mesh.
 */
void calMeshVal(){
    //Calculation about max,min
    maxX = meshMaxX();
    maxY = meshMaxY();
    maxZ = meshMaxZ();
    minX = meshMinX();
    minY = meshMinY();
    minZ = meshMinZ();
    meshCentreX = (maxX + minX)/2;
    meshCentreY = (maxY + minY)/2;
    meshCentreZ = (maxZ + minZ)/2;
    
    scaleX = maxX - meshCentreX;
    scaleY = maxY - meshCentreY;
    scaleZ = maxZ - meshCentreZ;
    
    // Calculation about scaleVector
    scaleVector = scaleX;
    if(scaleX < scaleY){
        scaleVector = scaleY;
    }
    if (scaleX < scaleZ) {
        scaleVector = scaleZ;
    }
    if (scaleY < scaleZ) {
        scaleVector = scaleZ;
    }
}
/*
 Applying transformation matrix (translate * scale) to each vertices.
 */
void transForm(){
    
    // Create the result transformation matrix (translate * scale)
    float TransM[4][4] = {{1/scaleVector,0,0,-meshCentreX*(1/scaleVector)},
                          {0,1/scaleVector,0,-meshCentreY*(1/scaleVector)},
                          {0,0,1/scaleVector,-meshCentreZ*(1/scaleVector)},
                          {0,0,0,1}};

    for (int i=0; i<vertices.size(); i++) {
        float x = vertices[i][0];
        float y = vertices[i][1];
        float z = vertices[i][2];
        vertices[i][0] = (TransM[0][0] * x) + (TransM[0][1] * y) + (TransM[0][2] * z) + TransM[0][3];
        vertices[i][1] = (TransM[1][0] * x) + (TransM[1][1] * y) + (TransM[1][2] * z) + TransM[1][3];
        vertices[i][2] = (TransM[2][0] * x) + (TransM[2][1] * y) + (TransM[2][2] * z) + TransM[2][3];
    }
}
/*
 Put calMeshVal() and transForm() together to form a single function.
 */
void scaleAndTranslate(){
    calMeshVal();
    transForm();
}
/*
 Drawing the loaded mesh
 */
void drawMesh(void)
{
    for (int i=0;i<faceIndices.size();i++)
    {
        glBegin(GL_TRIANGLES);
        glEnable(GL_NORMALIZE);
        
        //Create two vectors.
        float AB[3];
        float AC[3];
        float *temp;
        
        AB[0] = vertices[faceIndices[i][1]-1][0] - vertices[faceIndices[i][0]-1][0];
        AB[1] = vertices[faceIndices[i][1]-1][1] - vertices[faceIndices[i][0]-1][1];
        AB[2] = vertices[faceIndices[i][1]-1][2] - vertices[faceIndices[i][0]-1][2];
        
        AC[0] = vertices[faceIndices[i][2]-1][0] - vertices[faceIndices[i][0]-1][0];
        AC[1] = vertices[faceIndices[i][2]-1][1] - vertices[faceIndices[i][0]-1][1];
        AC[2] = vertices[faceIndices[i][2]-1][2] - vertices[faceIndices[i][0]-1][2];
    
        temp = calCrossProduct(AB, AC);
        
        glNormal3fv(temp);
        glVertex3f(vertices[faceIndices[i][0]-1][0], vertices[faceIndices[i][0]-1][1],
                   vertices[faceIndices[i][0]-1][2]);
        glNormal3fv(temp);
        glVertex3f(vertices[faceIndices[i][1]-1][0], vertices[faceIndices[i][1]-1][1],
                   vertices[faceIndices[i][1]-1][2]);
        glNormal3fv(temp);
        glVertex3f(vertices[faceIndices[i][2]-1][0], vertices[faceIndices[i][2]-1][1],
                   vertices[faceIndices[i][2]-1][2]);
        
        glEnd();
    }
}
/*
 Drawing Vertices of Mesh
 */
void drawMeshVertex(void)
{
    for (int i=0;i<faceIndices.size();i++)
    {
        glBegin(GL_POINTS);
        
        glVertex3f(vertices[faceIndices[i][0]-1][0], vertices[faceIndices[i][0]-1][1],
                   vertices[faceIndices[i][0]-1][2]);
        glVertex3f(vertices[faceIndices[i][1]-1][0], vertices[faceIndices[i][1]-1][1],
                   vertices[faceIndices[i][1]-1][2]);
        glVertex3f(vertices[faceIndices[i][2]-1][0], vertices[faceIndices[i][2]-1][1],
                   vertices[faceIndices[i][2]-1][2]);
        
        glEnd();
        glPointSize(1);
    }
}
/*
 Dawing mesh edges.
 */
void drawMeshEdges(void)
{
    for (int i=0;i<faceIndices.size();i++)
    {
        glBegin(GL_LINES);
        glVertex3f(vertices[faceIndices[i][0]-1][0], vertices[faceIndices[i][0]-1][1],
                   vertices[faceIndices[i][0]-1][2]);
        glVertex3f(vertices[faceIndices[i][1]-1][0], vertices[faceIndices[i][1]-1][1],
                   vertices[faceIndices[i][1]-1][2]);
        
        glVertex3f(vertices[faceIndices[i][1]-1][0], vertices[faceIndices[i][1]-1][1],
                   vertices[faceIndices[i][1]-1][2]);
        glVertex3f(vertices[faceIndices[i][2]-1][0], vertices[faceIndices[i][2]-1][1],
                   vertices[faceIndices[i][2]-1][2]);
        
        glVertex3f(vertices[faceIndices[i][2]-1][0], vertices[faceIndices[i][2]-1][1],
                   vertices[faceIndices[i][2]-1][2]);
        glVertex3f(vertices[faceIndices[i][0]-1][0], vertices[faceIndices[i][0]-1][1],
                   vertices[faceIndices[i][0]-1][2]);
        
        glEnd();
    }
}
/*
 Drawing cube with vertices only
 */
void drawVertex(){
    for (int i=0; i<6; i++) {
        glBegin(GL_POINTS);
        glColor3f(0.0f,1.0f,0.0f);

        for (int j=0; j<4; j++) {
            
            glVertex3fv(vertex[verticesForFace[i][j]]);

        }
        glEnd();
        glPointSize(5);
    }
}
/*
 Drawing cube with edges only.
 */
void drawEdges(){
    for (int i=0; i<6; i++) {
        glBegin(GL_LINES);
        glColor3f(0.0f,0.0f,1.0f);
        for (int j=0; j<8; j++) {
            glVertex3fv(vertex[edgesMap[i][j]]);
        }
        glEnd();
    }
}
/*
 Drawing the abb
 */
void drawAbb(){
    float vertexMesh[8][3] = {{maxX,maxY,maxZ},{minX,maxY,maxZ},{minX,minY,maxZ},{maxX,minY,maxZ},{maxX,minY,minZ},{minX,minY,minZ},{maxX,maxY,minZ},{minX,maxY,minZ}};
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for (int i=0; i<6; i++) {
        glBegin(GL_POLYGON);
        glColor4f(1.0f, 1.0f, 0.4f, 0.5f);
        for (int j =0; j<4; j++) {
            glVertex3fv(vertexMesh[verticesForFace[i][j]]);
        }
        glEnd();
    }
}

/*
 Work out the coordinates of obb points.
 */
void calOBBPoints(){
    eiVectorInverse = eiVector.transpose().inverse();
    for (int i =0; i<8; i++) {
        float x, y, z;

        x =obbPoints[i][0];
        y =obbPoints[i][1];
        z =obbPoints[i][2];

        obbPoints[i][0] = eiVectorInverse(0,0)*(x) + eiVectorInverse(0,1)*(y)+ eiVectorInverse(0,2)*(z) +calMean(0);
        obbPoints[i][1] = eiVectorInverse(1,0)*(x) + eiVectorInverse(1,1)*(y)+ eiVectorInverse(1,2)*(z) +calMean(1);
        obbPoints[i][2] = eiVectorInverse(2,0)*(x) + eiVectorInverse(2,1)*(y)+ eiVectorInverse(2,2)*(z) +calMean(2);
    }
}
/*
 Drawing the obb.
 */
void drawOBB(){
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for (int i=0; i<6; i++) {
        glBegin(GL_QUADS);
        glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
        for (int j =0; j<4; j++) {
            glVertex3fv(obbPoints[verticesForFace[i][j]]);
        }
        glEnd();
    }
}
/*
 * Scene initialisation
 */
void InitGL(GLvoid)
{
    glShadeModel(GL_SMOOTH);						// Enable smooth shading
    glClearColor(0.0f, 0.0f, 0.0f, 0.5f);			// Black background
    glClearDepth(1.0f);								// Depth buffer setup
    glEnable(GL_DEPTH_TEST);						// Enables depth testing
    glDepthFunc(GL_LEQUAL);						    // The type of depth testing to do
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

    
    GLfloat light_position[4] = { 1.0, 0.0, 1.0, 1.0 };
    GLfloat light_amb[4] = {0.1,0.1,0.1,1.0};
    GLfloat light_diff[4] = {0.5,0.5,0.5,1.0};
    GLfloat light_spec[4] = {0.2,0.8,0.2,1.0};
    
    GLfloat refl_amb[4] = {0.1, 0.8,0.1,1.0};
    GLfloat refl_diff[4] = {0.1, 0.8,0.1,1.0};
    GLfloat refl_spec[4] = {0.4, 0.4,0.4,1.0};
    GLfloat refl_shininess[] = {50};
    
    glMaterialfv(GL_FRONT, GL_AMBIENT, refl_amb);
    glMaterialfv(GL_FRONT, GL_SHININESS, refl_shininess);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,refl_diff);
    glMaterialfv(GL_FRONT, GL_SPECULAR,refl_spec);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    glLightfv( GL_LIGHT0, GL_POSITION, light_position );
    glLightfv( GL_LIGHT0, GL_AMBIENT, light_amb );
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_spec);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    
    setuptexture();
    glDisable(GL_TEXTURE_2D);
    
    glMatrixMode(GL_MODELVIEW);
 
}
void drawAxis(){
    glBegin(GL_LINES);
    //X AXIS
    glColor3f(0.7, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(3, 0, 0);
    //Y AXIS
    glColor3f(0, 0.5, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 3, 0);
    //Z AXIS
    glColor3f(0, 0, 0.7);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 4);
    glEnd();
}

void idle (void)
{
    
    glutPostRedisplay();   // trigger display callback
}

void display (void)
{
    int const width = glutGet(GLUT_WINDOW_WIDTH);
    int const height = glutGet(GLUT_WINDOW_HEIGHT);

    glClearColor (0.0, 0.0, 0.0, 0.0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Turn on 2D texturing
    glViewport(0,0,(GLsizei)width,(GLsizei)height);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    gluPerspective(fov,(GLfloat)width/(GLfloat)height,0.1f,100.0f);
    // Return to ModelView mode for future operations
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glBindTexture(GL_TEXTURE_2D, g_textureID[0]);
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    float *target;
    target= vectorAddition(cameraPos, cameraFront);
    //set normal camera
    gluLookAt(cameraPos[0], cameraPos[1], cameraPos[2],
              target[0], target[1], target[2],
              0.0f, 1.0f, 0.0f);

    
    // set the camera for orbiting aroud cub
    if (cameraRotation) {

        gluLookAt(eyex,eyey ,eyez ,
                  posx, posy, posz,
                  0.0f, 1.0f, 0.0f);
    }

    //Plotting Axis up front
    drawAxis();
    drawEdges();
    calMeshVal();
    //transForm();

    // different render mode*/
    switch ( rendermode ) {
        case 'f': // to display faces
            drawMesh();
            break;
        case 'v': // to display points
           // drawVertex();
            drawMeshVertex();
            break;
        case 'e': // to display edges
            drawMeshEdges();
            break;
        
    }
    if (showOBB) {drawOBB();}
    else{drawAbb();}
    glFlush();
    glutSwapBuffers();
}

/*
 * The reshape function sets up the viewport and projection
 */
void reshape ( int width, int height )
{
    // Prevent a divide by zero error by making height equal 1
    if (height==0)
    {
        height=1;
    }
    
    glViewport(0,0,width,height);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    // Need to calculate the aspect ratio of the window for gluPerspective
    gluPerspective(fov,(GLfloat)width/(GLfloat)height,0.1f,100.0f);
    // Return to ModelView mode for future operations
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/*
 * Callback for standard keyboard presses
 */
void keyboard ( unsigned char key, int x, int y )
{
    float moveSpeed = 0.5;
    switch(key) {
        // Exit the program when escape is pressed
        case 27:
            exit(0);
            break;
        
        // Switch render mode for v,e,f
        case 'v':
            rendermode='v';
            break;
            
        case 'e':
            rendermode='e';
            break;
            
        case 'f':
            rendermode='f';
            break;
        // Camera Motion
        case '[':
            theta-=10;//Set rotation angle
            eyez = radius*cos(theta*M_PI/180)*cos(alpha*M_PI/180);
            eyey = radius*-sin(alpha*M_PI/180);
            eyex = radius*-sin(theta*M_PI/180)*cos(alpha*M_PI/180);
            cameraRotation =1;
            break;
        case ']':
            theta+=10;
            eyez = radius*cos(theta*M_PI/180)*cos(alpha*M_PI/180);
            eyey = radius*-sin(alpha*M_PI/180);
            eyex = radius*-sin(theta*M_PI/180)*cos(alpha*M_PI/180);
            cameraRotation =1;
            break;
        case '9':
            alpha += 10;
            eyez = radius*cos(theta*M_PI/180)*cos(alpha*M_PI/180);
            eyey = radius*-sin(alpha*M_PI/180);
            eyex = radius*-sin(theta*M_PI/180)*cos(alpha*M_PI/180);
            cameraRotation =1;
            break;
            
        case '0':
            alpha -= 10;
            eyez = radius*cos(theta*M_PI/180)*cos(alpha*M_PI/180);
            eyey = radius*-sin(alpha*M_PI/180);
            eyex = radius*-sin(theta*M_PI/180)*cos(alpha*M_PI/180);
            cameraRotation =1;
            break;
  
        case'W':
            cameraPos[0] += moveSpeed*cameraFront[0];
            cameraPos[1] += moveSpeed*cameraFront[1];
            cameraPos[2] += moveSpeed*cameraFront[2];
            cameraRotation =0;

            break;
        case'S':
            cameraPos[0] -= moveSpeed*cameraFront[0];
            cameraPos[1] -= moveSpeed*cameraFront[1];
            cameraPos[2] -= moveSpeed*cameraFront[2];
            cameraRotation =0;

            break;
        case 'A':
            float *temp;
            temp = calCrossProduct(cameraFront,  cameraUp);
            cameraPos[0] -= moveSpeed*temp[0];
            cameraPos[1] -= moveSpeed*temp[1];
            cameraPos[2] -= moveSpeed*temp[2];
            cameraRotation =0;

            break;
        case 'D':
            float *temp2;
            temp2 = calCrossProduct(cameraFront,  cameraUp);
            cameraPos[0] += moveSpeed*temp2[0];
            cameraPos[1] += moveSpeed*temp2[1];
            cameraPos[2] += moveSpeed*temp2[2];
            cameraRotation =0;

            break;
            
            
        // Mesh operation
        case 's':
            rotateMeshX();
            break;;
        case 'a':
            rotateMeshY();
            //rotateY += 5;
            break;
        case 'q':
            rotateMeshZ();
            //rotateZ += 5;
            break;

            
            
        // Dis/En able lighting and texture.
        case 'o':
            glDisable(GL_LIGHTING);
            break;
        case 'p':
            glEnable(GL_LIGHTING);
            break;
        case 'T':
            glDisable(GL_TEXTURE_2D);
            break;
        case 't':
            glEnable(GL_TEXTURE_2D);
            break;
        // Enable/Disable abb and obb
        case ',':
            showABB = true;
            showOBB = false;
            break;
            
        case '.':
            showOBB = true;
            showABB = false;
            break;
        default:
            break;
    }
    
    glutPostRedisplay();
}



// Arrow keys need to be handled in a separate function from other keyboard presses
void arrow_keys ( int a_keys, int x, int y )
{
    switch ( a_keys ) {
        case GLUT_KEY_UP:
            glutFullScreen();
            break;
        case GLUT_KEY_DOWN:
            glutReshapeWindow(500, 500);
            break;
        default:
            break;
    }
}
/*
 Mouse button handled for zoom function
 */
void mouseButton(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON)
    {
        if (state == GLUT_DOWN) {
            mousePress = 1;
        }
        else{
            mousePress = 0;
        }

    }
}
/*
 change the value of fov field of view by value of Y.
 */
void mouseMove(int x, int y)
{
    if (mousePress)
    {
        fov = y/10;
        if (fov <= 1.0f)
            fov = 1.0f;

        if (fov >=45.0f) {
            fov = 45.0f;
        }
    }
    glutPostRedisplay();
}

/*
 * Entry point to the application
 */
int main(int argc, char** argv)
{
    // load the model.
    loadOBJ("bunny.obj", vertices, faceIndices);

    updateNormal();
    scaleAndTranslate();

    updateCovar();
    calOBBMM();
    calOBBPoints();

    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("CW2 OpenGL");
//    glutFullScreen();          // Uncomment to start in full screen
    InitGL();
    rendermode='f';
    
    // Callback functions
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(arrow_keys);  // For special keys
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMove);
    glutIdleFunc(idle);
    glutMainLoop();
    

}