```bash
docker swarm init --advertise-addr 10.5.98.250
```

This command will be displayed upon success: 
```bash
docker swarm join --advertise-addr 10.3.202.67 --token <token> 10.5.98.250:2377
``` 

## Deployment

Run this command in the `edge` node: 
```bash
docker compose -f docker-compose-edge.yml up -d
```

Run this command in the `robot` node: 
```bash
docker compose -f docker-compose-robot.yml up -d
```

To remove the stack, use: 
```bash
docker compose -f docker-compose-edge.yml down
```

```bash
docker compose -f docker-compose-robot.yml down
```





