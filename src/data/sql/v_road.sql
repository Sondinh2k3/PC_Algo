DROP TABLE IF EXISTS `v_road`;

CREATE TABLE `v_road` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `old_id` varchar(100) DEFAULT NULL,
  `flow_rate` float DEFAULT NULL,
  `occupancy_space` float DEFAULT NULL,
  `created_date` datetime NOT NULL,
  `duration` int DEFAULT NULL,
  `saturation_flow` int DEFAULT NULL,
  PRIMARY KEY (`id`)
)

INSERT INTO `v_road` (id, old_id, flow_rate, occupancy_space, created_date, duration, saturation_flow) VALUES 
    (1, 'road1', 1000.0, 0.5, '2023-10-01 10:00:00', 60, 2000),
    (2, 'road2', 1200.0, 0.6, '2023-10-01 10:05:00', 60, 2200),
    (3, 'road3', 800.0, 0.4, '2023-10-01 10:10:00', 60, 1800);