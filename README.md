# LLM-HybridSSD

The library needs to be configured with your account's secret key which is available on the website. It is recommended to set your OPENAI_API_KEY environment variable before using the library (or you can also set the API key directly in the code):

~~~
export OPENAI_API_KEY='sk-...'
mkdir build && cd build
cmake .. && make
examples/[whatever]
