#ifndef GRAPHICS_H
#define GRAPHICS_H

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <vector>
#include "IGfxEntity.h"
#include <DirectXColors.h>

#include <string>
using std::wstring;

namespace DirectX
{
	class SpriteBatch;
	class SpriteFont;
}

using DirectX::XMVECTORF32;
using DirectX::SpriteBatch;
using DirectX::SpriteFont;
using DirectX::XMFLOAT2;
using DirectX::XMFLOAT3;
using DirectX::XMFLOAT4;
using DirectX::XMVECTOR;

namespace Devil
{
	class D3D;
	class Camera;
	class Model;
	class ColorShader;
	class TextureShader;
	class LightShader;
	class Light;

	class GfxEntitySphere;
	class GfxEntityBox;
	class GfxEntityCylinder;
	class GfxEntityPlan;
	class GfxEntityHeightMap;
	class Texture;

	const float SCREEN_DEPTH = 100000.0f;
	const float SCREEN_NEAR = 0.1f;

	class Graphics
	{

	private:

		static Graphics* m_Instance;

		D3D* m_D3D;

		Camera* m_Camera;
		Model* m_Model;
		ColorShader* m_ColorShader;
		TextureShader* m_TextureShader;
		LightShader* m_LightShader;
		Light* m_Light;

		//List of graphics entities
		std::vector<IGfxEntity*> m_entityList;

		//List of textures
		std::vector<Texture*> m_textures;

		//Width of the screen in pixels
		int m_screenWidth;

		//Height of the screen in pixels.
		int m_screenHeight;

		//Define the color to use to clear the screen.
		XMVECTORF32 m_clearScreenColor;

		//DirectX object used to writ text on screen
		SpriteBatch* m_spriteBatch;

		//font consolas used to write text on screen
		SpriteFont* m_spriteFontConsolas;

		//Id of the box graphic entity
		unsigned int m_idBox;

		//Id of the sphere graphic entity
		unsigned int m_idSphere;

		//Id of the cylinder entity
		unsigned int m_idCylinder;

		//Id of the checker texture
		unsigned int m_IdTexChecker;

	public:
		static Graphics* getInstance();

		
		~Graphics();

		bool initialize(int _screenWidth, int _screenHeight, HWND _hwnd, bool _fullScreen, bool _vsync);
		
		void shutdown();
		
		void BeginRender();
		void EndRender();

		//Start rendering of sprites
		void spriteBeginRender();

		//end rendering of sprites.
		void spriteEndRender();

		// <summary>
		// Write text on screen
		// </summary>
		// <param name="_text">the text to write on screen</param>
		// <param name="_p">Position of the text on screen.</param>
		// <param name="_scale">Uniform scaling to apply to the text.</param>
		void writeText(const wstring& _text, const XMFLOAT2& _p, float _scale);

		GfxEntityPlan* createPlan(const XMFLOAT2&, const XMFLOAT4&);

		//Create an heightmap.
		// _lowerLeftcorner : position of the lower left corner of the terrain. The height is ignored. It is used to position the terrain
		//						in the world.
		// _quadSize : size of a quad.
		// _width : number of quads per row.
		// _length : number of quads per column.
		// _heights : array of float representing the heights of each vertex.
		// return : the id of the graphic entity.
		unsigned int createHeightMap(const XMVECTOR& _lowerLeftCorner, float _quadSize, unsigned int _width, unsigned int _length, 
			float* heights);

		D3D* getDirectXWrapper();
		Camera* getCamera();

		//Get the width of the screen in pixel.
		int getScreenWidth() const;

		//Get the height of the screen in pixel.
		int getScreenHeight() const;

		//Convert a world space position into a screen space position
		XMVECTOR worldToScreen(const XMVECTOR& _world);

		//Set the color to use to clear the screen.
		void setClearScreenColor(const XMVECTORF32& _color);

		//Delete all the graphics entity created
		void clear();

		//Delete an entity and clean its memory.
		// _id : id of the entity to release.
		void releaseEntity(unsigned int _id);

		IGfxEntity* getBox();

		IGfxEntity* getSphere();

		IGfxEntity* getCylinder();

		//Return a pointer to an entity. If the id does not correspond to an existing entity, an exception is thrown.
		// _id : id of the entity to return.
		// return : a pointer ot the entity.
		IGfxEntity* getEntity(unsigned int _id) const;

		Texture* getTexture(unsigned int _id);

		Texture* getTexChecker();

		//Overidde of the new operator to align this class to 16 bytes.
		// _count : the amount of memory to allocate in bytes.
		// return : a pointer to the allocated memory.
		void* operator new(size_t _count);

		//Override the delete operator. This class uses a 16 bytes alignment.
		// _p : a pointer to the memory to delete.
		void operator delete(void* _p);

	private:
		Graphics();

		//Create a box. 
		GfxEntityBox* createBox();

		GfxEntitySphere* createSphere();

		GfxEntityCylinder* createCylinder();
	};

#define GRAPHICS Graphics::getInstance()

}
#endif