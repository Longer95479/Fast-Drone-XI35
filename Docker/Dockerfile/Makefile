SPEED ?= 8

jetson_base:
	@docker build -t fastdronexi35:orin_base_35.3.1 -f ./Dockerfile.jetson_base --build-arg USE_PROC=$(SPEED) ..

jetson:
	@docker build -t fastdronexi35:orin -f ./Dockerfile.jetson --build-arg USE_PROC=$(SPEED) ..

pc:
	@docker build -t fastdronexi35:pc -f ./Dockerfile --build-arg USE_PROC=$(SPEED) ../..

rm_jetson:
	@docker rmi fastdronexi35:orin