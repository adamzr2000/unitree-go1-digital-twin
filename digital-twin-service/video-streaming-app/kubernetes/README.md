## Kubernetes Deployment

To deploy the application:

```bash
kubectl apply -f config.yml -f video-streaming-app.yml
```

🔗 **Access Stream:** [http://10.5.1.21:8889/go1_camera](http://10.5.1.21:8889/go1_camera) 

## Teardown

To delete the application:

```bash
kubectl delete -f config.yml -f video-streaming-app.yml
```