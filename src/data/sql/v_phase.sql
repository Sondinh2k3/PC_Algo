DROP TABLE IF EXISTS `v_phase`;

CREATE TABLE `v_phase` (
  `id` bigint NOT NULL AUTO_INCREMENT,
  `cross_id` bigint DEFAULT NULL,
  `min_green` int DEFAULT NULL,
  `max_green` int DEFAULT NULL,
  `order_number` int DEFAULT NULL,
  `phase_name` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) 

INSERT INTO `v_phase` (id, cross_id, min_green, max_green, order_number, phase_name) VALUES 
    (1, 1, 15, 100, 1, 'Pha 1'),
    (2, 1, 15, 100, 2, 'Pha 2'),
    (3, 1, 15, 100, 3, 'Pha 3'),
    (4, 2, 15, 100, 1, 'Pha 1'),
    (5, 2, 15, 100, 2, 'Pha 2'),
    (6, 2, 15, 100, 3, 'Pha 3');