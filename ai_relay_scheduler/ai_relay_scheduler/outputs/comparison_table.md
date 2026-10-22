规则 vs AI（INT8），同一批200个独立随机航程。

| 策略 | 断链事件 | 断链采样点 | 切换次数 | 平均余量(dB) | 乒乓次数 | 中继占用率 |
|---|---:|---:|---:|---:|---:|---:|
| 规则 | 20 | 31 | 1080 | 28.208 | 511 | 47.76% |
| AI | 0 | 0 | 811 | 29.064 | 282 | 53.63% |
| 规则/零延迟 | 0 | 0 | 1487 | 28.464 | 908 | 47.99% |
| AI/零延迟 | 0 | 0 | 954 | 29.106 | 423 | 53.57% |

切换增幅：-24.91%
配对统计：{'mean_outage_event_difference': -0.1, 'paired_bootstrap_95_ci': [-0.155, -0.05], 'one_sided_sign_test_p': 3.0517578125e-05, 'ai_better_voyages': 15, 'ai_worse_voyages': 0, 'tied_voyages': 185, 'bootstrap_repeats': 5000}
验收：{'significantly_fewer_outages': True, 'switch_growth_at_most_50_percent': True, 'ideal_zero_outages': True}
