const testBackendConnection = async () => {
  try {
    console.log("Testing backend connection to:", 'https://alizah-fatina-physical-robotic-book.hf.space/api/v1/agent/query');

    // Simulate the same request that the frontend would make
    const response = await fetch('https://alizah-fatina-physical-robotic-book.hf.space/api/v1/agent/query', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: 'Hello',
        selectedText: '',
        topK: 5
      })
    });

    console.log("Response status:", response.status);
    console.log("Response status text:", response.statusText);

    if (response.ok) {
      const data = await response.json();
      console.log("Response data:", JSON.stringify(data, null, 2));
    } else {
      console.log("Request failed with status:", response.status);
      const errorText = await response.text();
      console.log("Error response:", errorText);
    }
  } catch (error) {
    console.log("Network error:", error.message);
    console.log("This could be due to CORS restrictions when running from browser");
  }
};

// Note: This test may not work from Node.js due to CORS policies that apply in browsers
// The actual test should be done in the browser environment
testBackendConnection();