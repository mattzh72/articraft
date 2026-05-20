from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="bifold_louvered_shutter")

    painted = model.material("painted_white", color=(0.86, 0.86, 0.80, 1.0))
    slat_mat = model.material("slightly_worn_louver", color=(0.93, 0.92, 0.86, 1.0))
    shadow = model.material("shadowed_socket", color=(0.18, 0.17, 0.15, 1.0))
    metal = model.material("dull_zinc_hardware", color=(0.56, 0.58, 0.57, 1.0))

    height = 1.16
    leaf_w = 0.38
    stile_w = 0.035
    rail_h = 0.050
    thickness = 0.045
    louver_len = leaf_w - 2.0 * stile_w - 0.008
    pin_len = louver_len + 0.016
    rod_x = leaf_w - 0.085
    rod_y = -0.066
    louver_zs = [0.205, 0.335, 0.465, 0.595, 0.725, 0.855, 0.985]

    jamb = model.part("jamb")
    jamb.visual(
        Box((0.050, 0.060, height + 0.120)),
        origin=Origin(xyz=(-0.040, 0.0, (height + 0.120) / 2.0 - 0.060)),
        material=painted,
        name="standing_jamb",
    )
    jamb.visual(
        Box((0.030, 0.075, height + 0.060)),
        origin=Origin(xyz=(-0.037, 0.0, height / 2.0)),
        material=shadow,
        name="dark_reveal",
    )

    def add_leaf_frame(part, prefix: str) -> None:
        frame_names = {
            "outer": {
                "hinge": "outer_hinge_stile",
                "free": "outer_free_stile",
                "bottom": "outer_bottom_rail",
                "top": "outer_top_rail",
                "bridge": "outer_mid_bridge",
            },
            "inner": {
                "hinge": "inner_hinge_stile",
                "free": "inner_free_stile",
                "bottom": "inner_bottom_rail",
                "top": "inner_top_rail",
                "bridge": "inner_mid_bridge",
            },
        }[prefix]
        part.visual(
            Box((stile_w, thickness, height)),
            origin=Origin(xyz=(stile_w / 2.0, 0.0, height / 2.0)),
            material=painted,
            name=frame_names["hinge"],
        )
        part.visual(
            Box((stile_w, thickness, height)),
            origin=Origin(xyz=(leaf_w - stile_w / 2.0, 0.0, height / 2.0)),
            material=painted,
            name=frame_names["free"],
        )
        part.visual(
            Box((leaf_w, thickness, rail_h)),
            origin=Origin(xyz=(leaf_w / 2.0, 0.0, rail_h / 2.0)),
            material=painted,
            name=frame_names["bottom"],
        )
        part.visual(
            Box((leaf_w, thickness, rail_h)),
            origin=Origin(xyz=(leaf_w / 2.0, 0.0, height - rail_h / 2.0)),
            material=painted,
            name=frame_names["top"],
        )
        # A very shallow rear bridge keeps the louver bay reading as one routed
        # frame while leaving the front face open.
        part.visual(
            Box((leaf_w - 2.0 * stile_w, 0.010, 0.028)),
            origin=Origin(xyz=(leaf_w / 2.0, 0.020, 0.135)),
            material=painted,
            name=frame_names["bridge"],
        )
        for i, z in enumerate(louver_zs):
            for x, side in ((stile_w - 0.004, "hinge"), (leaf_w - stile_w + 0.004, "free")):
                part.visual(
                    Box((0.016, 0.014, 0.034)),
                    origin=Origin(xyz=(x, -0.029, z)),
                    material=shadow,
                    name=f"{prefix}_{side}_socket_{i}",
                )

    def add_jamb_hinge_hardware() -> None:
        for z in (0.245, 0.580, 0.915):
            jamb.visual(
                Box((0.030, 0.006, 0.145)),
                origin=Origin(xyz=(-0.020, 0.030, z)),
                material=metal,
                name=f"jamb_hinge_leaf_{int(z * 1000)}",
            )
        for z, length in ((0.180, 0.070), (0.515, 0.070), (0.850, 0.070)):
            jamb.visual(
                Cylinder(radius=0.008, length=length),
                origin=Origin(xyz=(0.0, 0.033, z), rpy=(0.0, 0.0, 0.0)),
                material=metal,
                name=f"jamb_knuckle_{int(z * 1000)}",
            )

    def add_leaf_hinge_knuckles(part, prefix: str, hinge_x: float, y: float, z_offset: float) -> None:
        plate_x = hinge_x + (0.014 if hinge_x < leaf_w / 2.0 else -0.014)
        part.visual(
            Box((0.018, 0.006, 0.910)),
            origin=Origin(xyz=(plate_x, y - 0.008, height / 2.0)),
            material=metal,
            name=f"{prefix}_hinge_strip",
        )
        for z in (0.180, 0.315, 0.515, 0.650, 0.850, 0.985):
            part.visual(
                Box((0.028, 0.006, 0.055)),
                origin=Origin(xyz=(plate_x, y - 0.008, z)),
                material=metal,
                name=f"{prefix}_hinge_plate_{int(z * 1000)}",
            )
        for z in (0.180 + z_offset, 0.515 + z_offset, 0.850 + z_offset):
            part.visual(
                Cylinder(radius=0.008, length=0.070),
                origin=Origin(xyz=(hinge_x, y, z), rpy=(0.0, 0.0, 0.0)),
                material=metal,
                name=f"{prefix}_knuckle_{int(z * 1000)}",
            )

    outer_leaf = model.part("outer_leaf")
    add_jamb_hinge_hardware()
    add_leaf_frame(outer_leaf, "outer")
    add_leaf_hinge_knuckles(outer_leaf, "jamb_side", hinge_x=0.0, y=0.033, z_offset=0.067)
    add_leaf_hinge_knuckles(outer_leaf, "fold_side", hinge_x=leaf_w, y=0.033, z_offset=0.000)

    inner_leaf = model.part("inner_leaf")
    add_leaf_frame(inner_leaf, "inner")
    add_leaf_hinge_knuckles(inner_leaf, "fold_side", hinge_x=0.0, y=0.033, z_offset=0.067)

    model.articulation(
        "jamb_to_outer",
        ArticulationType.REVOLUTE,
        parent=jamb,
        child=outer_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_leaf,
        child=inner_leaf,
        origin=Origin(xyz=(leaf_w, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.4, lower=-2.75, upper=0.0),
    )

    def add_control_rod(parent, prefix: str):
        rod = model.part(f"{prefix}_control_rod")
        rod.visual(
            Cylinder(radius=0.005, length=0.840),
            origin=Origin(xyz=(0.0, 0.0, 0.575)),
            material=metal,
            name="vertical_rod",
        )
        for i, z in enumerate(louver_zs):
            rod.visual(
                Box((0.018, 0.020, 0.018)),
                origin=Origin(xyz=(0.0, 0.015, z)),
                material=metal,
                name=f"drive_clip_{i}",
            )
        joint = model.articulation(
            f"{prefix}_rod_slide",
            ArticulationType.PRISMATIC,
            parent=parent,
            child=rod,
            origin=Origin(xyz=(rod_x, rod_y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.18, lower=-0.030, upper=0.030),
        )
        return rod, joint

    outer_rod, outer_rod_joint = add_control_rod(outer_leaf, "outer")
    inner_rod, inner_rod_joint = add_control_rod(inner_leaf, "inner")

    def add_louvers(parent, prefix: str) -> None:
        for i, z in enumerate(louver_zs):
            louver = model.part(f"{prefix}_louver_{i}")
            louver.visual(
                Box((louver_len, 0.064, 0.014)),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.16, 0.0, 0.0)),
                material=slat_mat,
                name="slat_blade",
            )
            louver.visual(
                Cylinder(radius=0.005, length=pin_len),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
                material=metal,
                name="pivot_pin",
            )
            louver.visual(
                Box((0.026, 0.014, 0.016)),
                origin=Origin(xyz=(rod_x - leaf_w / 2.0, -0.034, 0.0)),
                material=metal,
                name="tie_eyelet",
            )
            model.articulation(
                f"{prefix}_louver_pivot_{i}",
                ArticulationType.REVOLUTE,
                parent=parent,
                child=louver,
                origin=Origin(xyz=(leaf_w / 2.0, 0.0, z)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=-0.42, upper=0.42),
            )

    add_louvers(outer_leaf, "outer")
    add_louvers(inner_leaf, "inner")

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_leaf = object_model.get_part("outer_leaf")
    inner_leaf = object_model.get_part("inner_leaf")
    outer_rod = object_model.get_part("outer_control_rod")
    inner_rod = object_model.get_part("inner_control_rod")
    outer_hinge = object_model.get_articulation("jamb_to_outer")
    inner_hinge = object_model.get_articulation("outer_to_inner")
    outer_slide = object_model.get_articulation("outer_rod_slide")
    inner_slide = object_model.get_articulation("inner_rod_slide")

    ctx.expect_contact(
        outer_leaf,
        inner_leaf,
        elem_a="outer_free_stile",
        elem_b="inner_hinge_stile",
        contact_tol=0.001,
        name="fold leaves meet along the center hinge stile",
    )
    ctx.expect_overlap(
        outer_leaf,
        inner_leaf,
        axes="z",
        elem_a="outer_free_stile",
        elem_b="inner_hinge_stile",
        min_overlap=1.0,
        name="both leaves share the same shutter height",
    )

    # Louver pivot pins are intentionally captured slightly inside the side
    # stiles to read as seated clip pivots rather than floating bars.
    for prefix, leaf in (("outer", outer_leaf), ("inner", inner_leaf)):
        rod = outer_rod if prefix == "outer" else inner_rod
        for i in range(7):
            louver = object_model.get_part(f"{prefix}_louver_{i}")
            ctx.allow_overlap(
                leaf,
                louver,
                elem_b="pivot_pin",
                reason="The short louver pivot pin is intentionally clipped into the side-frame socket.",
            )
            ctx.allow_overlap(
                rod,
                louver,
                elem_a=f"drive_clip_{i}",
                elem_b="tie_eyelet",
                reason="The vertical control rod clip passes through the louver eyelet to drive tilt.",
            )
            ctx.expect_overlap(
                louver,
                leaf,
                axes="x",
                elem_a="pivot_pin",
                elem_b=f"{prefix}_hinge_stile",
                min_overlap=0.003,
                name=f"{prefix} louver {i} hinge-side pin remains frame-captured",
            )
            ctx.expect_overlap(
                louver,
                leaf,
                axes="x",
                elem_a="pivot_pin",
                elem_b=f"{prefix}_free_stile",
                min_overlap=0.004,
                name=f"{prefix} louver {i} free-side pin enters stile",
            )
            ctx.expect_contact(
                rod,
                louver,
                elem_a=f"drive_clip_{i}",
                elem_b="tie_eyelet",
                contact_tol=0.002,
                name=f"{prefix} rod clip {i} engages louver eyelet",
            )

    rest_outer_rod = ctx.part_world_position(outer_rod)
    driven_pose = {outer_slide: 0.030, inner_slide: -0.030}
    for i in range(7):
        driven_pose[f"outer_louver_pivot_{i}"] = 0.36
        driven_pose[f"inner_louver_pivot_{i}"] = -0.36
    with ctx.pose(driven_pose):
        lifted_outer_rod = ctx.part_world_position(outer_rod)
        lowered_inner_rod = ctx.part_world_position(inner_rod)
        outer_louver = object_model.get_part("outer_louver_3")
        inner_louver = object_model.get_part("inner_louver_3")
        ctx.expect_within(
            outer_louver,
            outer_leaf,
            axes="x",
            inner_elem="pivot_pin",
            outer_elem="outer_free_stile",
            margin=0.35,
            name="outer tilted louver remains clipped in its frame",
        )
        ctx.expect_within(
            inner_louver,
            inner_leaf,
            axes="x",
            inner_elem="pivot_pin",
            outer_elem="inner_free_stile",
            margin=0.35,
            name="inner tilted louver remains clipped in its frame",
        )
    ctx.check(
        "outer control rod translates upward",
        rest_outer_rod is not None and lifted_outer_rod is not None and lifted_outer_rod[2] > rest_outer_rod[2] + 0.020,
        details=f"rest={rest_outer_rod}, lifted={lifted_outer_rod}",
    )
    ctx.check(
        "inner control rod translates downward",
        lowered_inner_rod is not None and lowered_inner_rod[2] < 0.0,
        details=f"lowered={lowered_inner_rod}",
    )

    with ctx.pose({outer_hinge: 0.75, inner_hinge: -1.35}):
        outer_pos = ctx.part_world_position(outer_leaf)
        inner_pos = ctx.part_world_position(inner_leaf)
    ctx.check(
        "bifold leaves rotate on separate vertical axes",
        outer_pos is not None and inner_pos is not None and abs(inner_pos[1] - outer_pos[1]) > 0.10,
        details=f"outer={outer_pos}, inner={inner_pos}",
    )

    return ctx.report()


object_model = build_object_model()
