from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_shutter_set")

    painted_wood = Material("warm painted wood", rgba=(0.86, 0.82, 0.72, 1.0))
    end_grain = Material("exposed end grain", rgba=(0.72, 0.60, 0.43, 1.0))
    brass = Material("aged brass", rgba=(0.82, 0.62, 0.28, 1.0))
    shadow = Material("dark louver gaps", rgba=(0.12, 0.10, 0.08, 1.0))
    knob_mat = Material("ivory knob", rgba=(0.95, 0.90, 0.78, 1.0))

    leaf_w = 0.56
    leaf_h = 1.00
    stile_w = 0.055
    rail_h = 0.080
    panel_t = 0.060
    hinge_z = 0.75
    slat_count = 6
    slat_len = 0.420
    slat_depth = 0.110
    slat_thick = 0.018
    pin_len = 0.075
    pin_r = 0.009
    slat_pitch = 0.38
    rod_y = -0.082
    rod_x = leaf_w * 0.50
    louver_zs = [-0.315, -0.195, -0.075, 0.045, 0.165, 0.285]

    frame = model.part("window_frame")
    frame.visual(
        Box((0.080, 0.120, 1.700)),
        origin=Origin(xyz=(-0.640, 0.0, 0.850)),
        material=painted_wood,
        name="jamb_0",
    )
    frame.visual(
        Box((0.080, 0.120, 1.700)),
        origin=Origin(xyz=(0.640, 0.0, 0.850)),
        material=painted_wood,
        name="jamb_1",
    )
    frame.visual(
        Box((1.360, 0.120, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 1.658)),
        material=painted_wood,
        name="head_rail",
    )
    frame.visual(
        Box((1.360, 0.140, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=painted_wood,
        name="sill",
    )
    frame.visual(
        Box((1.240, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.055, 1.265)),
        material=end_grain,
        name="cafe_stop",
    )
    frame.visual(
        Box((1.240, 0.020, 0.900)),
        origin=Origin(xyz=(0.0, 0.060, 0.755)),
        material=shadow,
        name="opening_shadow",
    )

    def add_leaf(index: int, hinge_x: float, inward_sign: float) -> None:
        leaf = model.part(f"leaf_{index}")

        # The leaf frame origin is on its vertical hinge axis.  Positive local
        # X is still the model X direction, so mirrored leaves use signed
        # placement offsets rather than mirrored child frames.
        leaf.visual(
            Box((stile_w, panel_t, leaf_h)),
            origin=Origin(xyz=(inward_sign * (stile_w * 0.5), 0.0, 0.0)),
            material=painted_wood,
            name="outer_stile",
        )
        leaf.visual(
            Box((stile_w, panel_t, leaf_h)),
            origin=Origin(xyz=(inward_sign * (leaf_w - stile_w * 0.5), 0.0, 0.0)),
            material=painted_wood,
            name="inner_stile",
        )
        leaf.visual(
            Box((leaf_w, panel_t, rail_h)),
            origin=Origin(xyz=(inward_sign * (leaf_w * 0.5), 0.0, leaf_h * 0.5 - rail_h * 0.5)),
            material=painted_wood,
            name="top_rail",
        )
        leaf.visual(
            Box((leaf_w, panel_t, rail_h)),
            origin=Origin(xyz=(inward_sign * (leaf_w * 0.5), 0.0, -leaf_h * 0.5 + rail_h * 0.5)),
            material=painted_wood,
            name="bottom_rail",
        )
        for hinge_i, zc in enumerate((-0.280, 0.280)):
            leaf.visual(
                Cylinder(radius=0.012, length=0.210),
                origin=Origin(xyz=(-inward_sign * 0.002, -0.055, zc)),
                material=brass,
                name=f"hinge_barrel_{hinge_i}",
            )
            leaf.visual(
                Box((0.052, 0.026, 0.170)),
                origin=Origin(xyz=(inward_sign * 0.012, -0.040, zc)),
                material=brass,
                name=f"hinge_leaf_{hinge_i}",
            )

        model.articulation(
            f"leaf_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=leaf,
            origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
            axis=(0.0, 0.0, -inward_sign),
            motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.55),
        )

        rod = model.part(f"tilt_rod_{index}")
        rod.visual(
            Box((0.018, 0.014, 0.735)),
            origin=Origin(xyz=(inward_sign * rod_x, rod_y, 0.0)),
            material=brass,
            name="rod_bar",
        )
        for louver_i, zc in enumerate(louver_zs):
            rod.visual(
                Box((0.036, 0.044, 0.020)),
                origin=Origin(xyz=(inward_sign * rod_x, -0.053, zc)),
                material=brass,
                name=f"link_tab_{louver_i}",
            )
        model.articulation(
            f"tilt_rod_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=leaf,
            child=rod,
            origin=Origin(),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.25, lower=-0.035, upper=0.035),
        )

        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.034, length=0.026),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_mat,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.007, length=0.024),
            origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name="knob_stem",
        )
        knob.visual(
            Box((0.012, 0.007, 0.040)),
            origin=Origin(xyz=(0.0, -0.015, 0.014)),
            material=brass,
            name="grip_ridge",
        )
        model.articulation(
            f"knob_{index}_turn",
            ArticulationType.REVOLUTE,
            parent=rod,
            child=knob,
            origin=Origin(xyz=(inward_sign * rod_x, -0.125, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-math.pi, upper=math.pi),
        )

        for louver_i, zc in enumerate(louver_zs):
            louver = model.part(f"louver_{index}_{louver_i}")
            louver.visual(
                Box((slat_len, slat_depth, slat_thick)),
                origin=Origin(rpy=(slat_pitch, 0.0, 0.0)),
                material=painted_wood,
                name="slat",
            )
            louver.visual(
                Cylinder(radius=pin_r, length=pin_len),
                origin=Origin(xyz=(-0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brass,
                name="pin_0",
            )
            louver.visual(
                Cylinder(radius=pin_r, length=pin_len),
                origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brass,
                name="pin_1",
            )
            model.articulation(
                f"louver_{index}_{louver_i}_pivot",
                ArticulationType.REVOLUTE,
                parent=leaf,
                child=louver,
                origin=Origin(xyz=(inward_sign * leaf_w * 0.5, 0.0, zc)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=-0.42, upper=0.42),
            )

    add_leaf(0, -0.580, 1.0)
    add_leaf(1, 0.580, -1.0)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # The louver end pins are deliberately modeled as captured inside the side
    # stiles, while the tilt-rod tabs deliberately enter the front edge of each
    # slat like small staple links.  These are the local, hidden overlaps that
    # make the shutter read as a retained adjustable louver mechanism rather
    # than loose floating slats.
    for leaf_i in (0, 1):
        leaf = f"leaf_{leaf_i}"
        rod = f"tilt_rod_{leaf_i}"
        pin_to_stile = (
            (("pin_0", "outer_stile"), ("pin_1", "inner_stile"))
            if leaf_i == 0
            else (("pin_0", "inner_stile"), ("pin_1", "outer_stile"))
        )
        for louver_i in range(6):
            louver = f"louver_{leaf_i}_{louver_i}"
            for pin, stile in pin_to_stile:
                ctx.allow_overlap(
                    leaf,
                    louver,
                    elem_a=stile,
                    elem_b=pin,
                    reason="The louver end pivot pin is intentionally captured inside the side stile.",
                )
                ctx.expect_within(
                    louver,
                    leaf,
                    axes="yz",
                    inner_elem=pin,
                    outer_elem=stile,
                    margin=0.002,
                    name=f"{louver} {pin} is centered in {stile}",
                )
                ctx.expect_overlap(
                    louver,
                    leaf,
                    axes="x",
                    elem_a=pin,
                    elem_b=stile,
                    min_overlap=0.006,
                    name=f"{louver} {pin} remains inserted in {stile}",
                )

            ctx.allow_overlap(
                rod,
                louver,
                elem_a=f"link_tab_{louver_i}",
                elem_b="slat",
                reason="The tilt rod tab is an intentional small staple link into the louver face.",
            )
            ctx.expect_overlap(
                rod,
                louver,
                axes="yz",
                elem_a=f"link_tab_{louver_i}",
                elem_b="slat",
                min_overlap=0.006,
                name=f"{rod} tab {louver_i} engages {louver}",
            )

    ctx.check(
        "two leaves with louver controls",
        all(object_model.get_part(name) is not None for name in ("leaf_0", "leaf_1", "tilt_rod_0", "tilt_rod_1", "knob_0", "knob_1"))
        and all(object_model.get_part(f"louver_{leaf_i}_{louver_i}") is not None for leaf_i in (0, 1) for louver_i in range(6)),
        details="Expected two leaves, two tilt rods, two knobs, and six louver slats per leaf.",
    )

    frame = object_model.get_part("window_frame")
    leaf_0 = object_model.get_part("leaf_0")
    leaf_1 = object_model.get_part("leaf_1")
    ctx.expect_gap(
        leaf_1,
        leaf_0,
        axis="x",
        positive_elem="inner_stile",
        negative_elem="inner_stile",
        min_gap=0.020,
        max_gap=0.060,
        name="closed leaves meet with a narrow center reveal",
    )
    for leaf in (leaf_0, leaf_1):
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="sill",
            max_gap=0.002,
            max_penetration=0.001,
            name=f"{leaf.name} rests on the cafe sill line",
        )

    def _span(aabb, axis_index: int) -> float:
        return float(aabb[1][axis_index] - aabb[0][axis_index])

    for leaf_i in (0, 1):
        leaf = object_model.get_part(f"leaf_{leaf_i}")
        hinge = object_model.get_articulation(f"leaf_{leaf_i}_hinge")
        closed_aabb = ctx.part_element_world_aabb(leaf, elem="inner_stile")
        with ctx.pose({hinge: 1.20}):
            opened_aabb = ctx.part_element_world_aabb(leaf, elem="inner_stile")
        ctx.check(
            f"leaf_{leaf_i} swings outward on side hinge",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[0][1] < closed_aabb[0][1] - 0.20,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

        rod = object_model.get_part(f"tilt_rod_{leaf_i}")
        slide = object_model.get_articulation(f"tilt_rod_{leaf_i}_slide")
        rod_rest = ctx.part_world_position(rod)
        with ctx.pose({slide: 0.035}):
            rod_up = ctx.part_world_position(rod)
        ctx.check(
            f"tilt_rod_{leaf_i} translates vertically",
            rod_rest is not None and rod_up is not None and rod_up[2] > rod_rest[2] + 0.030,
            details=f"rest={rod_rest}, raised={rod_up}",
        )

        louver = object_model.get_part(f"louver_{leaf_i}_2")
        pivot = object_model.get_articulation(f"louver_{leaf_i}_2_pivot")
        slat_rest = ctx.part_element_world_aabb(louver, elem="slat")
        with ctx.pose({pivot: 0.42}):
            slat_tilted = ctx.part_element_world_aabb(louver, elem="slat")
        ctx.check(
            f"louver_{leaf_i}_2 rotates about its horizontal pivots",
            slat_rest is not None
            and slat_tilted is not None
            and _span(slat_tilted, 2) > _span(slat_rest, 2) + 0.010,
            details=f"rest={slat_rest}, tilted={slat_tilted}",
        )

        knob = object_model.get_part(f"knob_{leaf_i}")
        turn = object_model.get_articulation(f"knob_{leaf_i}_turn")
        ridge_rest = ctx.part_element_world_aabb(knob, elem="grip_ridge")
        with ctx.pose({turn: math.pi / 2.0}):
            ridge_turned = ctx.part_element_world_aabb(knob, elem="grip_ridge")
        ctx.check(
            f"knob_{leaf_i} rotates about its mounting axis",
            ridge_rest is not None
            and ridge_turned is not None
            and _span(ridge_turned, 0) > _span(ridge_rest, 0) + 0.015
            and _span(ridge_turned, 2) < _span(ridge_rest, 2) - 0.015,
            details=f"rest={ridge_rest}, turned={ridge_turned}",
        )

    return ctx.report()


object_model = build_object_model()
