# Waypoint input

Task 1--3 の waypoint YAML は、全 waypoint に WGS84 の `latitude` と
`longitude` を入力する。一つの座標形式だけであり、`origin`、`x`、`y`、
`coordinate_mode` は使わない。実行時に `/fromLL` が各点を `map` frameへ投影する。

```yaml
waypoints:
  - {id: 1, latitude: 34.5456666667, longitude: 135.5070555556, yaw: 0.0}
```

`njord-task start ... --auto` は `/fromLL` が利用可能になるまで開始を拒否する。
