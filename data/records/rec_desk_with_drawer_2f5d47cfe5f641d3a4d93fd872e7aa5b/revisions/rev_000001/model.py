from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_cabinet_desk")

    wood = model.material("warm_oak", rgba=(0.72, 0.45, 0.22, 1.0))
    edge_wood = model.material("dark_oak_edges", rgba=(0.42, 0.24, 0.10, 1.0))
    inner_shadow = model.material("drawer_shadow", rgba=(0.07, 0.055, 0.045, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.55, 0.52, 1.0))
    dark_steel = model.material("dark_slide_rails", rgba=(0.18, 0.18, 0.17, 1.0))
    brass = model.material("aged_brass_pulls", rgba=(0.78, 0.58, 0.25, 1.0))

    width = 1.10
    depth = 0.56
    height = 0.76
    panel_t = 0.040
    front_y = -depth / 2.0
    rear_y = depth / 2.0

    cabinet = model.part("cabinet")
    # Rectangular cabinet carcase: continuous side/top/bottom/back panels, plus
    # front rails around the drawer bay so the body reads as a desk cabinet and
    # remains one physically connected assembly.
    cabinet.visual(
        Box((panel_t, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + panel_t / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((panel_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - panel_t / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, panel_t, height)),
        origin=Origin(xyz=(0.0, rear_y - panel_t / 2.0, height / 2.0)),
        material=wood,
        name="rear_panel",
    )
    cabinet.visual(
        Box((width, depth, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_t / 2.0)),
        material=wood,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width, depth, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, height - panel_t / 2.0)),
        material=wood,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, 0.032, 0.062)),
        origin=Origin(xyz=(0.0, front_y + 0.016, 0.635)),
        material=edge_wood,
        name="top_front_rail",
    )
    cabinet.visual(
        Box((width, 0.032, 0.036)),
        origin=Origin(xyz=(0.0, front_y + 0.016, 0.398)),
        material=edge_wood,
        name="middle_front_rail",
    )
    cabinet.visual(
        Box((width, 0.032, 0.050)),
        origin=Origin(xyz=(0.0, front_y + 0.016, 0.125)),
        material=edge_wood,
        name="lower_front_rail",
    )
    cabinet.visual(
        Box((0.42, 0.010, 0.46)),
        origin=Origin(xyz=(0.0, rear_y - panel_t - 0.004, 0.365)),
        material=inner_shadow,
        name="dark_drawer_cavity",
    )

    drawer_centers_z = (0.515, 0.285)
    rail_length = 0.45
    rail_y = -0.015
    for drawer_index, zc in enumerate(drawer_centers_z):
        for side_index, x_sign in enumerate((-1.0, 1.0)):
            cabinet.visual(
                Box((0.080, rail_length, 0.022)),
                origin=Origin(
                    xyz=(
                        x_sign * (width / 2.0 - panel_t - 0.040),
                        rail_y,
                        zc - 0.085,
                    )
                ),
                material=dark_steel,
                name=f"drawer_{drawer_index}_guide_{side_index}",
            )

    # Long hinge leaves and alternating barrels at the front desk edge connect
    # the fold-down leaf mechanically to the carcase.
    hinge_y = front_y - 0.085
    hinge_z = height
    for i, x in enumerate((-0.42, 0.00, 0.42)):
        cabinet.visual(
            Box((0.18, abs(hinge_y - front_y), 0.006)),
            origin=Origin(xyz=(x, (hinge_y + front_y) / 2.0, height - 0.006)),
            material=steel,
            name=f"cabinet_hinge_leaf_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.012, length=0.18),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"cabinet_hinge_barrel_{i}",
        )
    cabinet.visual(
        Cylinder(radius=0.006, length=width - 0.08),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    leaf_t = 0.032
    leaf_depth = 0.46
    leaf = model.part("machine_leaf")
    leaf.visual(
        Box((width - 0.12, leaf_depth, leaf_t)),
        origin=Origin(xyz=(0.0, -(leaf_depth / 2.0 + 0.030), -leaf_t / 2.0)),
        material=wood,
        name="leaf_panel",
    )
    leaf.visual(
        Box((width - 0.10, 0.030, leaf_t + 0.006)),
        origin=Origin(xyz=(0.0, -(leaf_depth + 0.030), -leaf_t / 2.0)),
        material=edge_wood,
        name="front_edge_band",
    )
    leaf.visual(
        Box((0.030, leaf_depth, leaf_t + 0.004)),
        origin=Origin(xyz=(-(width - 0.12) / 2.0, -(leaf_depth / 2.0 + 0.030), -leaf_t / 2.0)),
        material=edge_wood,
        name="side_edge_band_0",
    )
    leaf.visual(
        Box((0.030, leaf_depth, leaf_t + 0.004)),
        origin=Origin(xyz=((width - 0.12) / 2.0, -(leaf_depth / 2.0 + 0.030), -leaf_t / 2.0)),
        material=edge_wood,
        name="side_edge_band_1",
    )
    for i, x in enumerate((-0.21, 0.21)):
        leaf.visual(
            Box((0.18, 0.045, 0.006)),
            origin=Origin(xyz=(x, -0.034, -0.006)),
            material=steel,
            name=f"leaf_hinge_leaf_{i}",
        )
        leaf.visual(
            Cylinder(radius=0.012, length=0.18),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"leaf_hinge_barrel_{i}",
        )
    leaf.visual(
        Box((width - 0.22, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, -0.070, -leaf_t - 0.010)),
        material=edge_wood,
        name="underside_stiffener",
    )

    model.articulation(
        "cabinet_to_machine_leaf",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=leaf,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=math.pi / 2.0),
    )

    def add_drawer(name: str, zc: float, index: int):
        drawer = model.part(name)
        drawer.visual(
            Box((0.86, 0.026, 0.190)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=wood,
            name="front_panel",
        )
        drawer.visual(
            Box((0.78, 0.012, 0.035)),
            origin=Origin(xyz=(0.0, -0.019, 0.000)),
            material=brass,
            name="recessed_pull",
        )
        drawer.visual(
            Box((0.790, 0.430, 0.016)),
            origin=Origin(xyz=(0.0, 0.215, -0.082)),
            material=edge_wood,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((0.020, 0.430, 0.135)),
            origin=Origin(xyz=(-0.405, 0.215, -0.020)),
            material=wood,
            name="drawer_side_0",
        )
        drawer.visual(
            Box((0.020, 0.430, 0.135)),
            origin=Origin(xyz=(0.405, 0.215, -0.020)),
            material=wood,
            name="drawer_side_1",
        )
        drawer.visual(
            Box((0.790, 0.020, 0.135)),
            origin=Origin(xyz=(0.0, 0.425, -0.020)),
            material=wood,
            name="drawer_back",
        )
        drawer.visual(
            Box((0.038, 0.430, 0.016)),
            origin=Origin(xyz=(-0.430, 0.215, -0.066)),
            material=dark_steel,
            name="runner_0",
        )
        drawer.visual(
            Box((0.038, 0.430, 0.016)),
            origin=Origin(xyz=(0.430, 0.215, -0.066)),
            material=dark_steel,
            name="runner_1",
        )
        model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(0.0, front_y - 0.020, zc)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.28),
        )
        return drawer

    add_drawer("drawer_0", drawer_centers_z[0], 0)
    add_drawer("drawer_1", drawer_centers_z[1], 1)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    leaf = object_model.get_part("machine_leaf")
    drawer_0 = object_model.get_part("drawer_0")
    drawer_1 = object_model.get_part("drawer_1")
    leaf_hinge = object_model.get_articulation("cabinet_to_machine_leaf")
    slide_0 = object_model.get_articulation("cabinet_to_drawer_0")
    slide_1 = object_model.get_articulation("cabinet_to_drawer_1")

    for barrel_index in (0, 1):
        barrel_name = f"leaf_hinge_barrel_{barrel_index}"
        ctx.allow_overlap(
            cabinet,
            leaf,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            reason="The steel hinge pin is intentionally captured inside the leaf hinge barrel.",
        )
        ctx.expect_within(
            cabinet,
            leaf,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=barrel_name,
            margin=0.001,
            name=f"hinge pin is coaxial with {barrel_name}",
        )
        ctx.expect_overlap(
            cabinet,
            leaf,
            axes="x",
            elem_a="hinge_pin",
            elem_b=barrel_name,
            min_overlap=0.16,
            name=f"hinge pin spans {barrel_name}",
        )

    ctx.expect_gap(
        cabinet,
        leaf,
        axis="y",
        min_gap=0.06,
        positive_elem="top_panel",
        negative_elem="leaf_panel",
        name="leaf is hinged beyond the front cabinet edge",
    )
    ctx.expect_gap(
        leaf,
        drawer_0,
        axis="z",
        min_gap=0.09,
        positive_elem="leaf_panel",
        negative_elem="front_panel",
        name="upper drawer sits below the sewing leaf",
    )
    ctx.expect_gap(
        drawer_0,
        drawer_1,
        axis="z",
        min_gap=0.03,
        positive_elem="front_panel",
        negative_elem="front_panel",
        name="two drawer fronts are vertically separated",
    )
    for drawer, guide_prefix in ((drawer_0, "drawer_0"), (drawer_1, "drawer_1")):
        ctx.expect_contact(
            drawer,
            cabinet,
            elem_a="runner_0",
            elem_b=f"{guide_prefix}_guide_0",
            contact_tol=0.003,
            name=f"{guide_prefix} left runner rides on its guide",
        )
        ctx.expect_contact(
            drawer,
            cabinet,
            elem_a="runner_1",
            elem_b=f"{guide_prefix}_guide_1",
            contact_tol=0.003,
            name=f"{guide_prefix} right runner rides on its guide",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="runner_0",
            elem_b=f"{guide_prefix}_guide_0",
            min_overlap=0.28,
            name=f"{guide_prefix} runner has closed insertion length",
        )

    leaf_open_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")
    with ctx.pose({leaf_hinge: math.pi / 2.0}):
        leaf_folded_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")
    open_leaf_center_z = (
        (leaf_open_aabb[0][2] + leaf_open_aabb[1][2]) / 2.0 if leaf_open_aabb else None
    )
    folded_leaf_center_z = (
        (leaf_folded_aabb[0][2] + leaf_folded_aabb[1][2]) / 2.0 if leaf_folded_aabb else None
    )
    ctx.check(
        "leaf hinge folds the work surface downward",
        open_leaf_center_z is not None
        and folded_leaf_center_z is not None
        and folded_leaf_center_z < open_leaf_center_z - 0.18,
        details=f"open_z={open_leaf_center_z}, folded_z={folded_leaf_center_z}",
    )

    for drawer, slide, guide_prefix in ((drawer_0, slide_0, "drawer_0"), (drawer_1, slide_1, "drawer_1")):
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: 0.28}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                elem_a="runner_0",
                elem_b=f"{guide_prefix}_guide_0",
                min_overlap=0.08,
                name=f"{guide_prefix} runner remains captured when extended",
            )
        ctx.check(
            f"{guide_prefix} slides outward from the cabinet",
            rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.22,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
