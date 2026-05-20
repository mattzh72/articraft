from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_panel_louvered_shutter")

    painted = model.material("warm_white_painted_wood", rgba=(0.88, 0.86, 0.78, 1.0))
    slat_mat = model.material("slightly_lighter_louvers", rgba=(0.95, 0.93, 0.84, 1.0))
    shadow = model.material("dark_recess_shadow", rgba=(0.08, 0.07, 0.06, 1.0))
    metal = model.material("brushed_dark_metal", rgba=(0.22, 0.22, 0.20, 1.0))

    # Realistic residential proportions: a broad window opening with two
    # plantation-shutter leaves hung from the outer jambs.
    leaf_width = 0.667
    leaf_height = 1.06
    stile_w = 0.062
    rail_h = 0.070
    leaf_depth = 0.052
    hinge_y = -0.058
    louver_y = -0.010
    rod_y = -0.086
    slat_zs = (-0.330, -0.198, -0.066, 0.066, 0.198, 0.330)
    rod_joint_limit = 0.040

    frame = model.part("window_frame")
    frame.visual(
        Box((0.080, 0.110, 1.320)),
        origin=Origin(xyz=(-0.740, 0.0, 0.0)),
        material=painted,
        name="jamb_0",
    )
    frame.visual(
        Box((0.080, 0.110, 1.320)),
        origin=Origin(xyz=(0.740, 0.0, 0.0)),
        material=painted,
        name="jamb_1",
    )
    frame.visual(
        Box((1.560, 0.110, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=painted,
        name="head_jamb",
    )
    frame.visual(
        Box((1.560, 0.110, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.640)),
        material=painted,
        name="sill",
    )
    frame.visual(
        Box((1.430, 0.012, 1.240)),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=Material("pale_window_glass", rgba=(0.70, 0.84, 0.95, 0.28)),
        name="glass_plane",
    )
    frame.visual(
        Box((0.030, 0.018, 1.240)),
        origin=Origin(xyz=(0.0, 0.058, 0.0)),
        material=painted,
        name="center_muntin",
    )

    # Fixed hinge knuckles and leaves on the two outer jambs.  Alternating
    # knuckle heights leave space for the moving barrels on the shutters.
    for side, hinge_x in (("0", -0.670), ("1", 0.670)):
        dir_sign = -1.0 if hinge_x < 0.0 else 1.0
        plate_x = hinge_x + dir_sign * 0.018
        for k, (zc, barrel_len) in enumerate(((-0.420, 0.160), (0.0, 0.180), (0.420, 0.160))):
            frame.visual(
                Box((0.055, 0.020, 0.125)),
                origin=Origin(xyz=(plate_x, -0.045, zc)),
                material=metal,
                name=f"fixed_hinge_plate_{side}_{k}",
            )
            frame.visual(
                Cylinder(radius=0.015, length=barrel_len),
                origin=Origin(xyz=(hinge_x, hinge_y, zc)),
                material=metal,
                name=f"fixed_hinge_barrel_{side}_{k}",
            )

    def add_leaf(leaf_index: int, direction: float, hinge_x: float) -> None:
        leaf = model.part(f"leaf_{leaf_index}")

        # Framed shutter leaf.  The part frame lies on the hinge axis; the leaf
        # geometry extends toward the center meeting line.
        outer_stile_x = direction * (stile_w / 2.0)
        center_stile_x = direction * (leaf_width - stile_w / 2.0)
        rail_center_x = direction * (leaf_width / 2.0)
        rail_len = leaf_width
        leaf.visual(
            Box((stile_w, leaf_depth, leaf_height)),
            origin=Origin(xyz=(outer_stile_x, 0.0, 0.0)),
            material=painted,
            name="outer_stile",
        )
        leaf.visual(
            Box((stile_w, leaf_depth, leaf_height)),
            origin=Origin(xyz=(center_stile_x, 0.0, 0.0)),
            material=painted,
            name="meeting_stile",
        )
        leaf.visual(
            Box((rail_len, leaf_depth, rail_h)),
            origin=Origin(xyz=(rail_center_x, 0.0, leaf_height / 2.0 - rail_h / 2.0)),
            material=painted,
            name="top_rail",
        )
        leaf.visual(
            Box((rail_len, leaf_depth, rail_h)),
            origin=Origin(xyz=(rail_center_x, 0.0, -leaf_height / 2.0 + rail_h / 2.0)),
            material=painted,
            name="bottom_rail",
        )

        # Thin shadowed side channels make the louver opening read as a true
        # recess rather than a flat filled panel.
        opening_center_x = direction * ((stile_w + (leaf_width - stile_w)) / 2.0)
        leaf.visual(
            Box((leaf_width - 2.0 * stile_w, 0.006, 0.018)),
            origin=Origin(xyz=(opening_center_x, 0.026, 0.455)),
            material=shadow,
            name="upper_louver_shadow",
        )
        leaf.visual(
            Box((leaf_width - 2.0 * stile_w, 0.006, 0.018)),
            origin=Origin(xyz=(opening_center_x, 0.026, -0.455)),
            material=shadow,
            name="lower_louver_shadow",
        )

        # Moving hinge knuckles mounted to the outer stile.
        for k, zc in enumerate((-0.210, 0.210)):
            leaf.visual(
                Box((0.086, 0.022, 0.115)),
                origin=Origin(xyz=(direction * 0.030, -0.033, zc)),
                material=metal,
                name=f"leaf_hinge_plate_{k}",
            )
            leaf.visual(
                Cylinder(radius=0.015, length=0.240),
                origin=Origin(xyz=(0.0, hinge_y, zc)),
                material=metal,
                name=f"leaf_hinge_barrel_{k}",
            )

        leaf_joint = model.articulation(
            f"frame_to_leaf_{leaf_index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=leaf,
            origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
            axis=(0.0, 0.0, -direction),
            motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=1.55),
        )
        leaf_joint.meta["qc_samples"] = [0.0, 0.45, 1.20]

        # Side-stile clevis clips at every slat height.  The slat pins run
        # between the upper/lower cheeks with visible clearance, so every slat
        # reads as mechanically captured without needing broad hidden overlap.
        outer_inner_face = direction * stile_w
        center_inner_face = direction * (leaf_width - stile_w)
        clip_len = 0.034
        clip_thick_z = 0.005
        clip_gap_z = 0.010
        side_clip_centers = (
            outer_inner_face + direction * clip_len / 2.0,
            center_inner_face - direction * clip_len / 2.0,
        )
        slat_center_x = sum(side_clip_centers) / 2.0
        pin_local = abs(side_clip_centers[1] - side_clip_centers[0]) / 2.0

        for row, zc in enumerate(slat_zs):
            for side_i, cx in enumerate(side_clip_centers):
                for half, zoff in (("upper", clip_gap_z / 2.0 + clip_thick_z / 2.0), ("lower", -clip_gap_z / 2.0 - clip_thick_z / 2.0)):
                    leaf.visual(
                        Box((clip_len, 0.018, clip_thick_z)),
                        origin=Origin(xyz=(cx, louver_y, zc + zoff)),
                        material=metal,
                        name=f"clip_{row}_{side_i}_{half}",
                    )

        rod = model.part(f"tilt_rod_{leaf_index}")
        rod.visual(
            Box((0.014, 0.012, 0.850)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=metal,
            name="vertical_bar",
        )
        for row, zc in enumerate(slat_zs):
            rod.visual(
                Box((0.020, 0.042, 0.006)),
                origin=Origin(xyz=(0.0, 0.015, zc)),
                material=metal,
                name=f"drive_tab_{row}",
            )

        rod_joint = model.articulation(
            f"leaf_{leaf_index}_to_tilt_rod",
            ArticulationType.PRISMATIC,
            parent=leaf,
            child=rod,
            origin=Origin(xyz=(slat_center_x, rod_y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.25, lower=-rod_joint_limit, upper=rod_joint_limit),
        )
        rod_joint.meta["qc_samples"] = [-rod_joint_limit, 0.0, rod_joint_limit]

        for row, zc in enumerate(slat_zs):
            slat = model.part(f"slat_{leaf_index}_{row}")
            slat.visual(
                Box((2.0 * pin_local - 0.020, 0.070, 0.012)),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=slat_mat,
                name="blade",
            )
            for side_i, px in enumerate((-pin_local, pin_local)):
                slat.visual(
                    Cylinder(radius=0.005, length=0.030),
                    origin=Origin(xyz=(px, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                    material=metal,
                    name=f"pivot_pin_{side_i}",
                )
            slat.visual(
                Box((0.020, 0.006, 0.026)),
                origin=Origin(xyz=(0.0, -0.037, 0.0)),
                material=metal,
                name="tilt_staple",
            )
            slat_joint = model.articulation(
                f"leaf_{leaf_index}_to_slat_{row}",
                ArticulationType.REVOLUTE,
                parent=leaf,
                child=slat,
                origin=Origin(xyz=(slat_center_x, louver_y, zc)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=-0.60, upper=0.60),
                mimic=(
                    Mimic(joint=f"leaf_{leaf_index}_to_slat_0", multiplier=1.0, offset=0.0)
                    if row > 0
                    else None
                ),
            )
            slat_joint.meta["qc_samples"] = [0.0]

    add_leaf(0, 1.0, -0.670)
    add_leaf(1, -1.0, 0.670)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("window_frame")
    leaf_0 = object_model.get_part("leaf_0")
    leaf_1 = object_model.get_part("leaf_1")
    leaf_hinge_0 = object_model.get_articulation("frame_to_leaf_0")
    leaf_hinge_1 = object_model.get_articulation("frame_to_leaf_1")
    rod_joint_0 = object_model.get_articulation("leaf_0_to_tilt_rod")
    rod_joint_1 = object_model.get_articulation("leaf_1_to_tilt_rod")
    slat_drive_0 = object_model.get_articulation("leaf_0_to_slat_0")
    slat_drive_1 = object_model.get_articulation("leaf_1_to_slat_0")

    ctx.check(
        "two shutter leaves hinge on vertical outer jamb axes",
        leaf_hinge_0.articulation_type == ArticulationType.REVOLUTE
        and leaf_hinge_1.articulation_type == ArticulationType.REVOLUTE
        and tuple(leaf_hinge_0.axis) == (0.0, 0.0, -1.0)
        and tuple(leaf_hinge_1.axis) == (0.0, 0.0, 1.0),
        details=f"axes={leaf_hinge_0.axis}, {leaf_hinge_1.axis}",
    )
    ctx.check(
        "each leaf has a vertical translating tilt rod",
        rod_joint_0.articulation_type == ArticulationType.PRISMATIC
        and rod_joint_1.articulation_type == ArticulationType.PRISMATIC
        and tuple(rod_joint_0.axis) == (0.0, 0.0, 1.0)
        and tuple(rod_joint_1.axis) == (0.0, 0.0, 1.0),
        details=f"axes={rod_joint_0.axis}, {rod_joint_1.axis}",
    )
    louver_joints = [
        object_model.get_articulation(f"leaf_{leaf_i}_to_slat_{row}")
        for leaf_i in (0, 1)
        for row in range(6)
    ]
    ctx.check(
        "every louver has a horizontal side-pivot revolute joint",
        len(louver_joints) == 12
        and all(j.articulation_type == ArticulationType.REVOLUTE and tuple(j.axis) == (1.0, 0.0, 0.0) for j in louver_joints),
        details=f"joint_count={len(louver_joints)}",
    )

    with ctx.pose({leaf_hinge_0: 0.0, leaf_hinge_1: 0.0}):
        ctx.expect_gap(
            leaf_1,
            leaf_0,
            axis="x",
            min_gap=0.0,
            max_gap=0.014,
            name="closed leaves meet at the center with a narrow reveal",
        )
        ctx.expect_within(
            leaf_0,
            frame,
            axes="xz",
            margin=0.002,
            name="leaf 0 sits inside the window opening",
        )
        ctx.expect_within(
            leaf_1,
            frame,
            axes="xz",
            margin=0.002,
            name="leaf 1 sits inside the window opening",
        )

    for leaf_i, leaf in ((0, leaf_0), (1, leaf_1)):
        rod = object_model.get_part(f"tilt_rod_{leaf_i}")
        for row in range(6):
            slat = object_model.get_part(f"slat_{leaf_i}_{row}")
            ctx.expect_contact(
                slat,
                leaf,
                contact_tol=0.002,
                name=f"slat {leaf_i}-{row} pivot pins are captured by stile clips",
            )
            ctx.expect_contact(
                slat,
                rod,
                elem_a="tilt_staple",
                elem_b=f"drive_tab_{row}",
                contact_tol=0.002,
                name=f"tilt rod tab reaches slat {leaf_i}-{row} staple",
            )

    def _aabb_height(part_name: str) -> float | None:
        bounds = ctx.part_world_aabb(object_model.get_part(part_name))
        if bounds is None:
            return None
        return bounds[1][2] - bounds[0][2]

    def _aabb_min_y(part_name: str) -> float | None:
        bounds = ctx.part_world_aabb(object_model.get_part(part_name))
        if bounds is None:
            return None
        return bounds[0][1]

    closed_leaf0_y = _aabb_min_y("leaf_0")
    closed_leaf1_y = _aabb_min_y("leaf_1")
    with ctx.pose({leaf_hinge_0: 0.85, leaf_hinge_1: 0.85}):
        open_leaf0_y = _aabb_min_y("leaf_0")
        open_leaf1_y = _aabb_min_y("leaf_1")
    ctx.check(
        "both leaves swing outward from the jamb hinges",
        closed_leaf0_y is not None
        and closed_leaf1_y is not None
        and open_leaf0_y is not None
        and open_leaf1_y is not None
        and open_leaf0_y < closed_leaf0_y - 0.20
        and open_leaf1_y < closed_leaf1_y - 0.20,
        details=f"closed_y=({closed_leaf0_y}, {closed_leaf1_y}), open_y=({open_leaf0_y}, {open_leaf1_y})",
    )

    rod0_rest_z = ctx.part_world_position(object_model.get_part("tilt_rod_0"))[2]
    rod1_rest_z = ctx.part_world_position(object_model.get_part("tilt_rod_1"))[2]
    slat0_rest_h = _aabb_height("slat_0_5")
    slat1_rest_h = _aabb_height("slat_1_5")
    with ctx.pose({rod_joint_0: 0.030, rod_joint_1: 0.030, slat_drive_0: 0.39, slat_drive_1: 0.39}):
        rod0_lift_z = ctx.part_world_position(object_model.get_part("tilt_rod_0"))[2]
        rod1_lift_z = ctx.part_world_position(object_model.get_part("tilt_rod_1"))[2]
        slat0_tilt_h = _aabb_height("slat_0_5")
        slat1_tilt_h = _aabb_height("slat_1_5")
    ctx.check(
        "tilt rods translate upward while louver stack rotates",
        rod0_lift_z > rod0_rest_z + 0.025
        and rod1_lift_z > rod1_rest_z + 0.025
        and slat0_rest_h is not None
        and slat1_rest_h is not None
        and slat0_tilt_h is not None
        and slat1_tilt_h is not None
        and slat0_tilt_h > slat0_rest_h + 0.015
        and slat1_tilt_h > slat1_rest_h + 0.015,
        details=(
            f"rod_z=({rod0_rest_z}->{rod0_lift_z}, {rod1_rest_z}->{rod1_lift_z}), "
            f"slat_h=({slat0_rest_h}->{slat0_tilt_h}, {slat1_rest_h}->{slat1_tilt_h})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
