# Complete Instructions: Connect Backend to Frontend

## Step 1: Set up Hugging Face Authentication

To push changes to your Hugging Face Space, you need to authenticate:

1. Go to https://huggingface.co/settings/tokens
2. Create a new token with `write` permissions
3. Configure git to use the token:

```bash
git remote set-url origin https://hf_xxx@huggingface.co/spaces/alizah-fatina/physical-humanoid-robotics-book.git
```
(Replace `hf_xxx` with your actual token)

## Step 2: Push Updated Backend to Hugging Face

```bash
cd physical-humanoid-robotics-book
git push
cd ..
```

## Step 3: Configure Environment Variables in Hugging Face Space

1. Go to: https://huggingface.co/spaces/alizah-fatina/physical-humanoid-robotics-book/settings
2. In the "Secrets" section, add these variables:
   - `OPENAI_API_KEY` - Your OpenAI API key
   - `COHERE_API_KEY` - Your Cohere API key
   - `QDRANT_URL` - Your Qdrant database URL
   - `QDRANT_API_KEY` - Your Qdrant API key

3. Make sure "Available in container" is checked for each
4. Restart your Space after adding secrets

## Step 4: Test Backend Availability

Wait a few minutes for the Space to restart with the new configuration, then test:

```bash
./test-backend-connection.sh
```

## Step 5: Build and Deploy Frontend

Once the backend is confirmed working:

```bash
./build-production.sh
npm run deploy
```

## Step 6: Verify Connection

1. Visit your GitHub Pages site
2. Open the chatbot
3. Send a test message
4. You should get a response (not "Failed to fetch")

## Troubleshooting

If you still get "Failed to fetch":

1. Check Hugging Face Space logs for specific errors
2. Verify all environment variables are set as secrets
3. Make sure the Space is running (not in error state)
4. Test the backend endpoint directly in a browser

## Success!

Your backend and frontend will be properly connected with no more "Failed to fetch" errors.