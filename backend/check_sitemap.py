"""
Utility script to check if the sitemap exists for the target site
"""
import requests

def check_sitemap():
    base_url = "https://alizah-fatima.github.io/physical-humanoid-ai-book/"
    sitemap_url = base_url + "sitemap.xml"

    try:
        response = requests.get(sitemap_url)
        if response.status_code == 200:
            print(f"Sitemap found at: {sitemap_url}")
            print(f"Content type: {response.headers.get('content-type')}")
            print(f"Content length: {len(response.content)} bytes")
            # Print first 500 characters to see the structure
            print(f"First 500 chars: {response.text[:500]}")
        else:
            print(f"No sitemap found at {sitemap_url} (status code: {response.status_code})")
    except Exception as e:
        print(f"Error checking sitemap: {str(e)}")

if __name__ == "__main__":
    check_sitemap()