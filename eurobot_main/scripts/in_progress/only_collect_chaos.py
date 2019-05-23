#!/usr/bin/env python


# move_chaos_to_red = bt.SequenceWithMemoryNode([
        #                         bt_ros.MoveLineToPoint(self.tactics.chaos_push_pose, "move_client"),
        #                         bt.ActionNode(lambda: self.score_master.add("BLUNIUM")),
        #                         bt.ActionNode(lambda: self.score_master.add("GREENIUM")),
        #                         bt.ActionNode(lambda: self.score_master.add("REDIUM")),
        #                         bt.ActionNode(lambda: self.score_master.add("REDIUM")),
        #
        #                         bt_ros.MoveLineToPoint(self.tactics.chaos_in_red_zone, "move_client"),
        #                         bt.ActionNode(lambda: self.score_master.unload("RED")),
        #                         bt.ActionNode(lambda: self.score_master.unload("RED")),
        #                         bt.ActionNode(lambda: self.score_master.unload("RED")),
        #                         bt.ActionNode(lambda: self.score_master.unload("RED"))
        #                     ])

        # move_to_chaos = bt.SequenceWithMemoryNode([
        #                     bt.ActionNode(self.update_main_coords),
        #                     bt.ActionNode(self.calculate_pucks_configuration),
        #                     bt.ActionNode(self.calculate_landings),
        #                     bt.ActionNode(self.choose_new_landing),
        #                     bt.ActionNode(self.calculate_prelanding),
        #
        #                     bt_ros.MoveLineToPoint(self.nearest_PRElanding, "move_client"),
        #                     bt_ros.MoveLineToPoint(self.nearest_landing, "move_client"),
        #                 ])
        #
        # # TODO take into account, that when colecting 7 pucks, don't make step down
        # collect_chaos = bt.SequenceNode([
        #                     bt.FallbackNode([
        #                         # completely?
        #                         bt.ConditionNode(self.is_chaos_collected_completely),
        #
        #                         # if last puck, just collect it, return success and get fuck out of here
        #                         bt.SequenceWithMemoryNode([
        #                             bt.ConditionNode(self.is_last_puck),
        #                             bt.SequenceWithMemoryNode([
        #                                 bt_ros.StartCollectGround("manipulator_client"),
        #                                 bt_ros.CompleteCollectGround("manipulator_client"),
        #                                 bt.ActionNode(self.update_chaos_pucks),
        #                                 bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color))
        #                             ])
        #                         ]),
        #
        #                         # calc config and start collect
        #                         bt.SequenceWithMemoryNode([
        #                             bt_ros.StartCollectGround("manipulator_client"),
        #                             bt.ActionNode(self.calculate_drive_back_point),
        #                             bt_ros.MoveLineToPoint(self.drive_back_point, "move_client"), # FIXME make it closer
        #
        #                             # calc new landing
        #                             bt.ActionNode(self.calculate_pucks_configuration),
        #                             bt.ActionNode(self.calculate_landings),
        #
        #                             # drive back, collect and move to new prelanding
        #                             bt.ParallelWithMemoryNode([
        #                                 bt_ros.CompleteCollectGround("manipulator_client"),
        #                                 bt.ActionNode(self.update_chaos_pucks),
        #                                 bt.ActionNode(lambda: self.score_master.add(self.incoming_puck_color)),
        #                                 bt_ros.MoveArcToPoint(self.nearest_PRElanding, "move_client"),
        #                             ], threshold=4),
        #
        #                             bt_ros.MoveLineToPoint(self.nearest_landing, "move_client"),
        #                         ]),
        #                     ]),
        #                     bt.ConditionNode(lambda: self.is_chaos_collected_completely1())
        #                 ])
