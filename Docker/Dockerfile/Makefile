SPEED ?= 32

jetson_base:
	@docker build -t fastdronexi35:orin_base_35.3.1 -f ./Dockerfile.jetson_base --build-arg USE_PROC=$(SPEED) ..

jetson:
	@docker build -t fastdronexi35:orin -f ./Dockerfile.jetson --build-arg USE_PROC=$(SPEED) ..

jetson_rebuild:
	@docker build --no-cache -t fastdronexi35:orin -f ./Dockerfile.jetson --build-arg USE_PROC=$(SPEED) ..

pc:
	@echo "⚠️ Please download TensorRT and put it in the parent directory (../) url:https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.6.1/tars/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz"
	@bash -c 'read -n 1 -s -r -p "Press any key to continue..."; echo ""'
	@HTTP_PROXY="http://127.0.0.1:7897"; \
	HTTPS_PROXY="http://127.0.0.1:7897"; \
	USE_PROXY=false; \
	read -p "Do you want to use a proxy? [y/n] " ans; \
	ans=$${ans:-y}; \
	if [ "$$ans" = "y" ]; then \
		USE_PROXY=true; \
		read -p "Use default proxy ($$HTTP_PROXY) ($$HTTPS_PROXY)? [y/n] " def; \
		def=$${def:-y}; \
		if [ "$$def" = "n" ]; then \
			read -p "Enter HTTP proxy: " HTTP_PROXY; \
			read -p "Enter HTTPS proxy: " HTTPS_PROXY; \
		fi; \
	fi; \
	echo "Use proxy: $$USE_PROXY"; \
	echo "HTTP proxy: $$HTTP_PROXY"; \
	echo "HTTPS proxy: $$HTTPS_PROXY"; \
	docker build --network=host -t fastdronexi35:pc -f ./Dockerfile.pc \
		--build-arg USE_PROC=$(SPEED) \
		--build-arg USE_PROXY=$$USE_PROXY \
		--build-arg HTTP_PROXY=$$HTTP_PROXY \
		--build-arg HTTPS_PROXY=$$HTTPS_PROXY \
		..

rm_jetson:
	@docker rmi fastdronexi35:orin