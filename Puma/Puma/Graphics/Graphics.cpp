#include "Graphics.h"

Graphics::~Graphics()
{
}

bool Graphics::Initialize(HWND hwnd, int width, int height)
{
	this->windowWidth = width;
	this->windowHeight = height;
	this->fpsTimer.Start();

	if (!InitializeDirectX(hwnd))
		return false;

	if (!InitializeShaders())
		return false;

	if (!InitializeScene())
		return false;

	InitGui(hwnd);

	return true;
}
void Graphics::RenderFrame()
{
	float bgcolor[] = { 0.05f, 0.05f, 0.1f, 1.0f };
	this->deviceContext->ClearRenderTargetView(this->renderTargetView.Get(), bgcolor);
	this->deviceContext->ClearDepthStencilView(this->depthStencilView.Get(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	RenderVisualisation();
	RenderGui();

	this->swapchain->Present(0, NULL);
}

void Graphics::InitGui(HWND hwnd) {
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	ImGui_ImplWin32_Init(hwnd);
	ImGui_ImplDX11_Init(this->device.Get(), this->deviceContext.Get());
	ImGui::StyleColorsDark();
}

void Graphics::RenderGui() {
	ImGui_ImplDX11_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	RenderMainPanel();

	ImGui::Render();
	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}
void Graphics::RenderMainPanel() {
	ImGui::SetNextWindowSize(ImVec2(500, 950), ImGuiCond_Once);
	ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_Once);
	if (!ImGui::Begin("Main Panel"))
	{
		ImGui::End();
		return;
	}

	if (simulation->paused) {
		if (ImGui::Button("Start"))
			simulation->paused = false;
	}
	else {
		if (ImGui::Button("Pause"))
			simulation->paused = true;
	}

	ImGui::SameLine();
	if (ImGui::Button("Reset")) 
		simulation->Reset();
		

	ImGui::Separator();
	ImGui::SliderFloat3("start position", &simulation->robot.startState.Position.x, 0, 5);
	ImGui::SliderFloat3("end position", &simulation->robot.endState.Position.x, 0, 5);

	ImGui::Separator();

	static Vector3 startRotation = simulation->robot.ToDeg(simulation->robot.QtoE(simulation->robot.startState.Rotation));
	if(ImGui::SliderFloat3("start rotation Euler", &startRotation.x, -180, 180, "%.0f"))
		simulation->robot.startState.Rotation = simulation->robot.EtoQ(simulation->robot.ToRad(startRotation));

	static Vector3 endRotation = simulation->robot.ToDeg(simulation->robot.QtoE(simulation->robot.endState.Rotation));
	if(ImGui::SliderFloat3("end rotation Euler", &endRotation.x, -180, 180, "%.0f"))
		simulation->robot.endState.Rotation = simulation->robot.EtoQ(simulation->robot.ToRad(endRotation));

	ImGui::Separator();

	ImGui::Checkbox("loop", &simulation->loop);
	ImGui::Checkbox("inner CS", &simulation->robot.showInnerCS);
	ImGui::SliderFloat("animation time", &simulation->animationTime, 1, 5);
	ImGui::SliderFloat("animation progress", &simulation->time, 0, simulation->animationTime);

	ImGui::End();
}

void Graphics::RenderVisualisation()
{
	this->deviceContext->IASetInputLayout(this->vertexshader.GetInputLayout());
	this->deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	this->deviceContext->RSSetState(this->rasterizerState.Get());
	this->deviceContext->OMSetDepthStencilState(this->depthStencilState.Get(), 0);

	this->deviceContext->VSSetShader(vertexshader.GetShader(), NULL, 0);
	this->deviceContext->PSSetShader(pixelshader.GetShader(), NULL, 0);
	this->deviceContext->VSSetConstantBuffers(0, 1, this->cbColoredObject.GetAddressOf());
	this->deviceContext->PSSetConstantBuffers(0, 1, this->cbColoredObject.GetAddressOf());

	//top
	this->deviceContext->RSSetViewports(1, &viewportTop);
	RenderCS(simulation->robot.startState.GetMatrix());
	RenderCS(simulation->robot.endState.GetMatrix());

	RenderPuma(true);


	//down
	this->deviceContext->RSSetViewports(1, &viewportDown);
	RenderCS(simulation->robot.startState.GetMatrix());
	RenderCS(simulation->robot.endState.GetMatrix());

	//RenderPuma(false);
	RenderPuma(true);

}
void Graphics::RenderCylinder(Matrix worldMatrix, Vector4 color)
{
	UINT offset = 0;

	cbColoredObject.data.worldMatrix = worldMatrix;
	cbColoredObject.data.wvpMatrix = cbColoredObject.data.worldMatrix * camera.GetViewMatrix() * camera.GetProjectionMatrix();
	cbColoredObject.data.color = color;

	cbColoredObject.ApplyChanges();
	this->deviceContext->IASetVertexBuffers(0, 1, vbCylinder.GetAddressOf(), vbCylinder.StridePtr(), &offset);
	this->deviceContext->IASetIndexBuffer(ibCylinder.Get(), DXGI_FORMAT_R32_UINT, 0);
	this->deviceContext->DrawIndexed(ibCylinder.BufferSize(), 0, 0);

}
void Graphics::RenderPuma(bool angleInterpolation)
{
	InnerState state = simulation->robot.GetState(simulation->time / simulation->animationTime, angleInterpolation);
	//InnerState state = simulation->robot.GetState(0, angleInterpolation);
	vector<Matrix> css = simulation->robot.GetMatrixes(state);

	float width = 0.2;
	Matrix cubeMtx = Matrix::CreateTranslation(-0.5f, -0.5f, 0) * Matrix::CreateScale(width, width, 1);
	RenderCube(cubeMtx * Matrix::CreateScale(1, 1, simulation->robot.l[0]) * css[0], { 1,1,1,1 });
	RenderCube(cubeMtx * Matrix::CreateScale(1, 1, state.q) * Matrix::CreateRotationY(XM_PIDIV2) * css[1], { 1,1,1,1 });
	RenderCube(cubeMtx * Matrix::CreateScale(1, 1, simulation->robot.l[1]) * Matrix::CreateTranslation(0, 0, -simulation->robot.l[1]) * css[2], { 1,1,1,1 });
	RenderCube(cubeMtx * Matrix::CreateScale(1, 1, simulation->robot.l[2]) * Matrix::CreateRotationY(XM_PIDIV2) * css[3], { 1,1,1,1 });

	Matrix cylinderMtx = Matrix::CreateScale(0.3);
	RenderCylinder(cylinderMtx * css[0], { 1,1,1,1 });
	RenderCylinder(cylinderMtx * Matrix::CreateRotationX(XM_PIDIV2) * css[1], { 1,1,1,1 });
	RenderCylinder(cylinderMtx * Matrix::CreateRotationX(XM_PIDIV2) * css[2], { 1,1,1,1 });
	RenderCylinder(cylinderMtx * css[3], { 1,1,1,1 });
	RenderCylinder(Matrix::CreateTranslation(0, 0, -0.5f) * cylinderMtx * Matrix::CreateRotationY(XM_PIDIV2) * css[4], { 1,1,1,1 });

	if (simulation->robot.showInnerCS)
	{
		Matrix scale = Matrix::CreateScale(0.6f);
		RenderCS(scale * css[0]);
		RenderCS(scale * css[1]);
		RenderCS(scale * css[2]);
		RenderCS(scale * css[3]);
		RenderCS(scale * css[4]);
	}
}
void Graphics::RenderCS(Matrix worldMatrix)
{
	float width = 0.03f;
	RenderCube(Matrix::CreateScale(width, width, width) * worldMatrix, { 0.3f, 0.3f, 0.3f, 1 });
	RenderCube(Matrix::CreateTranslation(width, 0, 0) * Matrix::CreateScale(1, width, width) * worldMatrix, { 1,0,0,1 });
	RenderCube(Matrix::CreateTranslation(0, width, 0) * Matrix::CreateScale(width, 1, width) * worldMatrix, { 0,1,0,1 });
	RenderCube(Matrix::CreateTranslation(0, 0, width) * Matrix::CreateScale(width, width, 1) * worldMatrix, { 0,0,1,1 });
}
void Graphics::RenderCube(Matrix worldMatrix, Vector4 color)
{
	UINT offset = 0;

	cbColoredObject.data.worldMatrix = worldMatrix;
	cbColoredObject.data.wvpMatrix = cbColoredObject.data.worldMatrix * camera.GetViewMatrix() * camera.GetProjectionMatrix();
	cbColoredObject.data.color = color;

	cbColoredObject.ApplyChanges();
	this->deviceContext->IASetVertexBuffers(0, 1, vbCube.GetAddressOf(), vbCube.StridePtr(), &offset);
	this->deviceContext->IASetIndexBuffer(ibCube.Get(), DXGI_FORMAT_R32_UINT, 0);
	this->deviceContext->DrawIndexed(ibCube.BufferSize(), 0, 0);
}

bool Graphics::InitializeDirectX(HWND hwnd)
{
	std::vector<AdapterData> adapters = AdapterReader::GetAdapters();

	if (adapters.size() < 1)
	{
		ErrorLogger::Log("No IDXGI Adapters found.");
		return false;
	}

	DXGI_SWAP_CHAIN_DESC scd;
	ZeroMemory(&scd, sizeof(DXGI_SWAP_CHAIN_DESC));

	scd.BufferDesc.Width = this->windowWidth;
	scd.BufferDesc.Height = this->windowHeight;
	scd.BufferDesc.RefreshRate.Numerator = 60;
	scd.BufferDesc.RefreshRate.Denominator = 1;
	scd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	scd.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	scd.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;

	scd.SampleDesc.Count = 1;
	scd.SampleDesc.Quality = 0;

	scd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	scd.BufferCount = 1;
	scd.OutputWindow = hwnd;
	scd.Windowed = TRUE;
	scd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
	scd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

	HRESULT hr;
	hr = D3D11CreateDeviceAndSwapChain(adapters[0].pAdapter, //IDXGI Adapter
		D3D_DRIVER_TYPE_UNKNOWN,
		NULL, //FOR SOFTWARE DRIVER TYPE
		NULL, //FLAGS FOR RUNTIME LAYERS
		NULL, //FEATURE LEVELS ARRAY
		0, //# OF FEATURE LEVELS IN ARRAY
		D3D11_SDK_VERSION,
		&scd, //Swapchain description
		this->swapchain.GetAddressOf(), //Swapchain Address
		this->device.GetAddressOf(), //Device Address
		NULL, //Supported feature level
		this->deviceContext.GetAddressOf()); //Device Context Address

	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create device and swapchain.");
		return false;
	}

	Microsoft::WRL::ComPtr<ID3D11Texture2D> backBuffer;
	hr = this->swapchain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(backBuffer.GetAddressOf()));
	if (FAILED(hr)) //If error occurred
	{
		ErrorLogger::Log(hr, "GetBuffer Failed.");
		return false;
	}

	hr = this->device->CreateRenderTargetView(backBuffer.Get(), NULL, this->renderTargetView.GetAddressOf());
	if (FAILED(hr)) //If error occurred
	{
		ErrorLogger::Log(hr, "Failed to create render target view.");
		return false;
	}

	//Describe our Depth/Stencil Buffer
	D3D11_TEXTURE2D_DESC depthStencilDesc;
	depthStencilDesc.Width = this->windowWidth;
	depthStencilDesc.Height = this->windowHeight;
	depthStencilDesc.MipLevels = 1;
	depthStencilDesc.ArraySize = 1;
	depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	depthStencilDesc.SampleDesc.Count = 1;
	depthStencilDesc.SampleDesc.Quality = 0;
	depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
	depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	depthStencilDesc.CPUAccessFlags = 0;
	depthStencilDesc.MiscFlags = 0;

	hr = this->device->CreateTexture2D(&depthStencilDesc, NULL, this->depthStencilBuffer.GetAddressOf());
	if (FAILED(hr)) //If error occurred
	{
		ErrorLogger::Log(hr, "Failed to create depth stencil buffer.");
		return false;
	}

	hr = this->device->CreateDepthStencilView(this->depthStencilBuffer.Get(), NULL, this->depthStencilView.GetAddressOf());
	if (FAILED(hr)) //If error occurred
	{
		ErrorLogger::Log(hr, "Failed to create depth stencil view.");
		return false;
	}

	this->deviceContext->OMSetRenderTargets(1, this->renderTargetView.GetAddressOf(), this->depthStencilView.Get());

	//Create depth stencil state
	D3D11_DEPTH_STENCIL_DESC depthstencildesc;
	ZeroMemory(&depthstencildesc, sizeof(D3D11_DEPTH_STENCIL_DESC));

	depthstencildesc.DepthEnable = true;
	depthstencildesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK::D3D11_DEPTH_WRITE_MASK_ALL;
	depthstencildesc.DepthFunc = D3D11_COMPARISON_FUNC::D3D11_COMPARISON_LESS_EQUAL;

	hr = this->device->CreateDepthStencilState(&depthstencildesc, this->depthStencilState.GetAddressOf());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create depth stencil state.");
		return false;
	}

	//Create the Viewport
	viewportHeight = this->windowHeight / 2;

	ZeroMemory(&viewportTop, sizeof(D3D11_VIEWPORT));
	viewportTop.TopLeftX = 0;
	viewportTop.TopLeftY = 0;
	viewportTop.Width = this->windowWidth;
	viewportTop.Height = viewportHeight;
	viewportTop.MinDepth = 0.0f;
	viewportTop.MaxDepth = 1.0f;

	ZeroMemory(&viewportDown, sizeof(D3D11_VIEWPORT));
	viewportDown.TopLeftX = 0;
	viewportDown.TopLeftY = viewportHeight;
	viewportDown.Width = this->windowWidth;
	viewportDown.Height = viewportHeight;
	viewportDown.MinDepth = 0.0f;
	viewportDown.MaxDepth = 1.0f;

	////Set the Viewport
	//this->deviceContext->RSSetViewports(1, &viewportRight);

	//Create Rasterizer State
	D3D11_RASTERIZER_DESC rasterizerDesc;
	ZeroMemory(&rasterizerDesc, sizeof(D3D11_RASTERIZER_DESC));

	rasterizerDesc.FillMode = D3D11_FILL_MODE::D3D11_FILL_SOLID;
	rasterizerDesc.CullMode = D3D11_CULL_MODE::D3D11_CULL_BACK;
	hr = this->device->CreateRasterizerState(&rasterizerDesc, this->rasterizerState.GetAddressOf());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create rasterizer state.");
		return false;
	}

	spriteBatch = std::make_unique<DirectX::SpriteBatch>(this->deviceContext.Get());
	spriteFont = std::make_unique<DirectX::SpriteFont>(this->device.Get(), L"Data\\Fonts\\comic_sans_ms_16.spritefont");


	return true;
}
bool Graphics::InitializeShaders()
{
	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{"POSITION", 0, DXGI_FORMAT::DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_CLASSIFICATION::D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{"NORMAL", 0, DXGI_FORMAT::DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_CLASSIFICATION::D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};

	UINT numElements = ARRAYSIZE(layout);

	if (!vertexshader.Initialize(this->device, L"my_vs.cso", layout, numElements))
		return false;

	if (!pixelshader.Initialize(this->device, L"my_ps.cso"))
		return false;

	if (!diagonalPixelshader.Initialize(this->device, L"diagonal_ps.cso"))
		return false;

	return true;
}
bool Graphics::InitializeScene()
{
	InitCube();
	InitCylinder();

	//Initialize Constant Buffer(s)
	this->cbColoredObject.Initialize(this->device.Get(), this->deviceContext.Get());
	this->cbLight.Initialize(this->device.Get(), this->deviceContext.Get());


	camera.SetPosition(0, -5.0f, 0);
	camera.SetProjectionValues(60.0f, static_cast<float>(windowWidth) / static_cast<float>(viewportHeight), 0.1f, 1000.0f);

	return true;
}
void Graphics::InitCube()
{
	VertexPN v[] = {
		VertexPN(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f),
		VertexPN(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f),
		VertexPN(1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f),
		VertexPN(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f),

		VertexPN(1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f),
		VertexPN(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f),
		VertexPN(0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f),
		VertexPN(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f),

		VertexPN(1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f),
		VertexPN(1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f),
		VertexPN(1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f),
		VertexPN(1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f),

		VertexPN(0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f),
		VertexPN(0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f),
		VertexPN(0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f),
		VertexPN(0.0f, 1.0f, 1.0f, -1.0f, 0.0f, 0.0f),

		VertexPN(0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f),
		VertexPN(1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f),
		VertexPN(1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f),
		VertexPN(0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f),

		VertexPN(0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f),
		VertexPN(1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f),
		VertexPN(1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f),
		VertexPN(0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f)
	};

	int indices[] =
	{
		0, 2, 3, 0,1, 2,
		4, 6, 7, 4,5, 6,
		8, 10, 11, 8, 9, 10,
		12, 14, 15, 12, 13, 14,
		16, 18, 19, 16, 17, 18,
		20, 22, 23, 20, 21, 22,
	};

	this->vbCube.Initialize(this->device.Get(), v, ARRAYSIZE(v));
	this->ibCube.Initialize(this->device.Get(), indices, ARRAYSIZE(indices));

}
void Graphics::InitCylinder()
{
	float r = 0.5f;
	int horizontalLvls = 25;

	vector<VertexPN> vertices;
	vector<int> indices;

	for (int i = 0; i < horizontalLvls; i++)
	{
		float angle = XM_2PI * i / horizontalLvls;
		float angle2 = XM_2PI * (i + 1) / horizontalLvls;
		Vector3 a = Vector3(cos(angle), sin(angle), 0);
		Vector3 b = Vector3(cos(angle2), sin(angle2), 0);

		a.Normalize();
		b.Normalize();

		int count = vertices.size();
		vertices.push_back(VertexPN(r * a.x, r * a.y, -0.5f, a.x, a.y, 0));
		vertices.push_back(VertexPN(r * a.x, r * a.y, 0.5f, a.x, a.y, 0));
		vertices.push_back(VertexPN(r * b.x, r * b.y, 0.5f, b.x, b.y, 0));
		vertices.push_back(VertexPN(r * b.x, r * b.y, -0.5f, b.x, b.y, 0));

		indices.push_back(count); indices.push_back(count + 1); indices.push_back(count + 2);
		indices.push_back(count); indices.push_back(count + 2); indices.push_back(count + 3);
	}

	for (int i = 0; i < horizontalLvls; i++)
	{
		float angle = XM_2PI * i / horizontalLvls;
		float angle2 = XM_2PI * (i + 1) / horizontalLvls;
		Vector3 a = Vector3(cos(angle), sin(angle), 0);
		Vector3 b = Vector3(cos(angle2), sin(angle2), 0);

		a.Normalize();
		b.Normalize();

		int count = vertices.size();
		vertices.push_back(VertexPN(0, 0, 0.5f, 0, 0, 1));
		vertices.push_back(VertexPN(r * b.x, r * b.y, 0.5f, 0, 0, 1));
		vertices.push_back(VertexPN(r * a.x, r * a.y, 0.5f, 0, 0, 1));
		vertices.push_back(VertexPN(0, 0, -0.5f, 0, 0, -1));
		vertices.push_back(VertexPN(r * a.x, r * a.y, -0.5f, 0, 0, -1));
		vertices.push_back(VertexPN(r * b.x, r * b.y, -0.5f, 0, 0, -1));

		indices.push_back(count); indices.push_back(count + 1); indices.push_back(count + 2);
		indices.push_back(count + 3); indices.push_back(count + 4); indices.push_back(count + 5);
	}

	this->vbCylinder.Initialize(this->device.Get(), vertices.data(), vertices.size());
	this->ibCylinder.Initialize(this->device.Get(), indices.data(), indices.size());
}
