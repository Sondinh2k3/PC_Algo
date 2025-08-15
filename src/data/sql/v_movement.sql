DROP TABLE IF EXISTS `v_movement`;

CREATE TABLE `v_movement` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `phase_id` bigint DEFAULT NULL,
  `from_edge_id` varchar(100) CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci DEFAULT NULL,
  `to_edge_id` varchar(100) CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci DEFAULT NULL,
  `proportion` float DEFAULT NULL,
  PRIMARY KEY (`id`)
)

INSERT INTO `v_movement` (id, phase_id, from_edge_id, to_edge_id, proportion) VALUES 
    (1, 1, 'edge1', 'edge2', 0.5),
    (2, 1, 'edge2', 'edge3', 0.5),
    (3, 2, 'edge3', 'edge4', 0.6),
    (4, 2, 'edge4', 'edge5', 0.4),
    (5, 3, 'edge5', 'edge6', 0.7),
    (6, 3, 'edge6', 'edge7', 0.3);