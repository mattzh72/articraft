from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


PALM_DISTAL_Z = 0.082
FINGER_NAMES = ("index", "middle", "ring", "little")
FINGER_SPECS = (
    {
        "name": "index",
        "root_x": -0.028,
        "base": {"width": 0.017, "thickness": 0.020, "length": 0.016, "joint_radius": 0.0050},
        "segments": (
            {"name": "proximal", "width": 0.016, "thickness": 0.018, "length": 0.039, "joint_radius": 0.0048},
            {"name": "middle", "width": 0.014, "thickness": 0.016, "length": 0.025, "joint_radius": 0.0042},
            {"name": "distal", "width": 0.012, "thickness": 0.014, "length": 0.019, "joint_radius": 0.0038},
        ),
        "splay_limits": (-0.30, 0.16),
    },
    {
        "name": "middle",
        "root_x": -0.008,
        "base": {"width": 0.018, "thickness": 0.021, "length": 0.017, "joint_radius": 0.0052},
        "segments": (
            {"name": "proximal", "width": 0.017, "thickness": 0.019, "length": 0.043, "joint_radius": 0.0050},
            {"name": "middle", "width": 0.015, "thickness": 0.017, "length": 0.028, "joint_radius": 0.0044},
            {"name": "distal", "width": 0.013, "thickness": 0.015, "length": 0.020, "joint_radius": 0.0039},
        ),
        "splay_limits": (-0.14, 0.14),
    },
    {
        "name": "ring",
        "root_x": 0.011,
        "base": {"width": 0.017, "thickness": 0.020, "length": 0.016, "joint_radius": 0.0050},
        "segments": (
            {"name": "proximal", "width": 0.016, "thickness": 0.018, "length": 0.041, "joint_radius": 0.0048},
            {"name": "middle", "width": 0.014, "thickness": 0.016, "length": 0.026, "joint_radius": 0.0042},
            {"name": "distal", "width": 0.012, "thickness": 0.014, "length": 0.019, "joint_radius": 0.0038},
        ),
        "splay_limits": (-0.12, 0.20),
    },
    {
        "name": "little",
        "root_x": 0.029,
        "base": {"width": 0.015, "thickness": 0.019, "length": 0.015, "joint_radius": 0.0046},
        "segments": (
            {"name": "proximal", "width": 0.014, "thickness": 0.017, "length": 0.034, "joint_radius": 0.0043},
            {"name": "middle", "width": 0.012, "thickness": 0.015, "length": 0.021, "joint_radius": 0.0038},
            {"name": "distal", "width": 0.010, "thickness": 0.013, "length": 0.017, "joint_radius": 0.0034},
        ),
        "splay_limits": (-0.12, 0.30),
    },
)
THUMB_SPEC = {
    "name": "thumb",
    "mount_xyz": (0.040, 0.0, 0.018),
    "mount_yaw": 0.88,
    "base": {"width": 0.020, "thickness": 0.022, "length": 0.018, "joint_radius": 0.0054},
    "segments": (
        {"name": "proximal", "width": 0.018, "thickness": 0.020, "length": 0.033, "joint_radius": 0.0048},
        {"name": "distal", "width": 0.015, "thickness": 0.017, "length": 0.025, "joint_radius": 0.0041},
    ),
    "splay_limits": (-0.45, 0.45),
}


def _section(width: float, thickness: float, z: float) -> list[tuple[float, float, float]]:
    radius = max(0.0008, min(width, thickness) * 0.24)
    return [(x, y, z) for x, y in rounded_rect_profile(width, thickness, radius, corner_segments=6)]


def _tapered_mesh(
    name: str,
    *,
    width: float,
    thickness: float,
    length: float,
    mid_scale: float = 0.92,
    tip_scale: float = 0.62,
) -> object:
    geometry = section_loft(
        (
            _section(width, thickness, 0.0),
            _section(width * mid_scale, thickness * 0.94, length * 0.72),
            _section(max(width * tip_scale, 0.0045), max(thickness * 0.68, 0.0036), length),
        )
    )
    return mesh_from_geometry(geometry, name)


def _add_digit_link(
    part,
    *,
    shell_mesh,
    shell_material,
    joint_material,
    pad_material,
    width: float,
    thickness: float,
    length: float,
    joint_radius: float,
    joint_axis: str,
    pad_length_scale: float = 0.46,
    pad_center_fraction: float = 0.31,
    pad_y_fraction: float = -0.10,
) -> None:
    if joint_axis == "x":
        barrel_origin = Origin(xyz=(0.0, 0.0, joint_radius), rpy=(0.0, math.pi / 2.0, 0.0))
        barrel_length = width * 0.96
    else:
        barrel_origin = Origin(xyz=(0.0, 0.0, joint_radius), rpy=(math.pi / 2.0, 0.0, 0.0))
        barrel_length = thickness * 0.96

    part.visual(
        Cylinder(radius=joint_radius, length=barrel_length),
        origin=barrel_origin,
        material=joint_material,
        name="joint_barrel",
    )
    part.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, joint_radius * 0.50)),
        material=shell_material,
        name="shell",
    )
    pad_length = max(length * pad_length_scale, 0.006)
    part.visual(
        Box((width * 0.72, thickness * 0.24, pad_length)),
        origin=Origin(
            xyz=(
                0.0,
                thickness * pad_y_fraction,
                joint_radius + max(length * pad_center_fraction, pad_length * 0.5),
            )
        ),
        material=pad_material,
        name="pad",
    )
    part.inertial = Inertial.from_geometry(
        Box((width, thickness, length)),
        mass=max(0.012, width * thickness * length * 1800.0),
        origin=Origin(xyz=(0.0, 0.0, length * 0.5)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anthropomorphic_robotic_palm")

    palm_shell = model.material("palm_shell", rgba=(0.24, 0.26, 0.29, 1.0))
    digit_shell = model.material("digit_shell", rgba=(0.68, 0.71, 0.75, 1.0))
    joint_dark = model.material("joint_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    pad_dark = model.material("pad_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    accent = model.material("accent", rgba=(0.45, 0.52, 0.58, 1.0))

    palm = model.part("palm")
    palm_mesh = mesh_from_geometry(
        section_loft(
            (
                _section(0.062, 0.032, -0.028),
                _section(0.082, 0.039, 0.006),
                _section(0.092, 0.037, 0.050),
                _section(0.096, 0.031, PALM_DISTAL_Z),
            )
        ),
        "palm_shell_mesh",
    )
    palm.visual(palm_mesh, material=palm_shell, name="shell")
    palm.visual(
        Box((0.050, 0.006, 0.044)),
        origin=Origin(xyz=(0.000, -0.021, 0.032)),
        material=pad_dark,
        name="palm_pad",
    )
    palm.visual(
        Box((0.050, 0.004, 0.018)),
        origin=Origin(xyz=(0.000, 0.0185, 0.060)),
        material=accent,
        name="dorsal_rib",
    )
    palm.visual(
        Box((0.046, 0.028, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, -0.026)),
        material=joint_dark,
        name="wrist_mount",
    )
    for spec in FINGER_SPECS:
        palm.visual(
            Box((spec["base"]["width"] + 0.006, 0.028, 0.008)),
            origin=Origin(xyz=(spec["root_x"], 0.0, PALM_DISTAL_Z - 0.004)),
            material=joint_dark,
            name=f"{spec['name']}_seat",
        )
    palm.visual(
        Box((0.008, 0.026, 0.012)),
        origin=Origin(xyz=(0.044, 0.0, 0.013)),
        material=joint_dark,
        name="thumb_seat",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.096, 0.040, 0.110)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    for spec in FINGER_SPECS:
        name = spec["name"]
        base_dims = spec["base"]
        base_part = model.part(f"{name}_base")
        base_shell = _tapered_mesh(
            f"{name}_base_shell",
            width=base_dims["width"],
            thickness=base_dims["thickness"],
            length=base_dims["length"] - base_dims["joint_radius"] * 0.50,
            mid_scale=0.95,
            tip_scale=0.78,
        )
        _add_digit_link(
            base_part,
            shell_mesh=base_shell,
            shell_material=digit_shell,
            joint_material=joint_dark,
            pad_material=pad_dark,
            width=base_dims["width"],
            thickness=base_dims["thickness"],
            length=base_dims["length"],
            joint_radius=base_dims["joint_radius"],
            joint_axis="y",
        )
        model.articulation(
            f"{name}_splay",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=base_part,
            origin=Origin(xyz=(spec["root_x"], 0.0, PALM_DISTAL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=2.0,
                lower=spec["splay_limits"][0],
                upper=spec["splay_limits"][1],
            ),
        )

        proximal_spec, middle_spec, distal_spec = spec["segments"]
        proximal = model.part(f"{name}_proximal")
        _add_digit_link(
            proximal,
            shell_mesh=_tapered_mesh(
                f"{name}_proximal_shell",
                width=proximal_spec["width"],
                thickness=proximal_spec["thickness"],
                length=proximal_spec["length"] - proximal_spec["joint_radius"] * 0.50,
            ),
            shell_material=digit_shell,
            joint_material=joint_dark,
            pad_material=pad_dark,
            width=proximal_spec["width"],
            thickness=proximal_spec["thickness"],
            length=proximal_spec["length"],
            joint_radius=proximal_spec["joint_radius"],
            joint_axis="x",
        )
        model.articulation(
            f"{name}_knuckle_flex",
            ArticulationType.REVOLUTE,
            parent=base_part,
            child=proximal,
            origin=Origin(xyz=(0.0, 0.0, base_dims["length"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.45),
        )

        middle = model.part(f"{name}_middle")
        _add_digit_link(
            middle,
            shell_mesh=_tapered_mesh(
                f"{name}_middle_shell",
                width=middle_spec["width"],
                thickness=middle_spec["thickness"],
                length=middle_spec["length"] - middle_spec["joint_radius"] * 0.50,
            ),
            shell_material=digit_shell,
            joint_material=joint_dark,
            pad_material=pad_dark,
            width=middle_spec["width"],
            thickness=middle_spec["thickness"],
            length=middle_spec["length"],
            joint_radius=middle_spec["joint_radius"],
            joint_axis="x",
        )
        model.articulation(
            f"{name}_middle_flex",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, 0.0, proximal_spec["length"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.6, velocity=2.5, lower=0.0, upper=1.55),
        )

        distal = model.part(f"{name}_distal")
        _add_digit_link(
            distal,
            shell_mesh=_tapered_mesh(
                f"{name}_distal_shell",
                width=distal_spec["width"],
                thickness=distal_spec["thickness"],
                length=distal_spec["length"] - distal_spec["joint_radius"] * 0.50,
                mid_scale=0.88,
                tip_scale=0.52,
            ),
            shell_material=digit_shell,
            joint_material=joint_dark,
            pad_material=pad_dark,
            width=distal_spec["width"],
            thickness=distal_spec["thickness"],
            length=distal_spec["length"],
            joint_radius=distal_spec["joint_radius"],
            joint_axis="x",
        )
        model.articulation(
            f"{name}_distal_flex",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, 0.0, middle_spec["length"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=1.25),
        )

    thumb_base_dims = THUMB_SPEC["base"]
    thumb_base = model.part("thumb_base")
    _add_digit_link(
        thumb_base,
        shell_mesh=_tapered_mesh(
            "thumb_base_shell",
            width=thumb_base_dims["width"],
            thickness=thumb_base_dims["thickness"],
            length=thumb_base_dims["length"] - thumb_base_dims["joint_radius"] * 0.50,
            mid_scale=0.95,
            tip_scale=0.78,
        ),
        shell_material=digit_shell,
        joint_material=joint_dark,
        pad_material=pad_dark,
        width=thumb_base_dims["width"],
        thickness=thumb_base_dims["thickness"],
        length=thumb_base_dims["length"],
        joint_radius=thumb_base_dims["joint_radius"],
        joint_axis="y",
        pad_length_scale=0.22,
        pad_center_fraction=0.13,
        pad_y_fraction=-0.16,
    )
    model.articulation(
        "thumb_splay",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_base,
        origin=Origin(xyz=THUMB_SPEC["mount_xyz"], rpy=(0.0, THUMB_SPEC["mount_yaw"], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=THUMB_SPEC["splay_limits"][0],
            upper=THUMB_SPEC["splay_limits"][1],
        ),
    )

    thumb_proximal_spec, thumb_distal_spec = THUMB_SPEC["segments"]
    thumb_proximal = model.part("thumb_proximal")
    _add_digit_link(
        thumb_proximal,
        shell_mesh=_tapered_mesh(
            "thumb_proximal_shell",
            width=thumb_proximal_spec["width"],
            thickness=thumb_proximal_spec["thickness"],
            length=thumb_proximal_spec["length"] - thumb_proximal_spec["joint_radius"] * 0.50,
        ),
        shell_material=digit_shell,
        joint_material=joint_dark,
        pad_material=pad_dark,
        width=thumb_proximal_spec["width"],
        thickness=thumb_proximal_spec["thickness"],
        length=thumb_proximal_spec["length"],
        joint_radius=thumb_proximal_spec["joint_radius"],
        joint_axis="x",
    )
    model.articulation(
        "thumb_knuckle_flex",
        ArticulationType.REVOLUTE,
        parent=thumb_base,
        child=thumb_proximal,
        origin=Origin(xyz=(0.0, 0.0, thumb_base_dims["length"])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.5, lower=0.0, upper=1.25),
    )

    thumb_distal = model.part("thumb_distal")
    _add_digit_link(
        thumb_distal,
        shell_mesh=_tapered_mesh(
            "thumb_distal_shell",
            width=thumb_distal_spec["width"],
            thickness=thumb_distal_spec["thickness"],
            length=thumb_distal_spec["length"] - thumb_distal_spec["joint_radius"] * 0.50,
            mid_scale=0.88,
            tip_scale=0.54,
        ),
        shell_material=digit_shell,
        joint_material=joint_dark,
        pad_material=pad_dark,
        width=thumb_distal_spec["width"],
        thickness=thumb_distal_spec["thickness"],
        length=thumb_distal_spec["length"],
        joint_radius=thumb_distal_spec["joint_radius"],
        joint_axis="x",
    )
    model.articulation(
        "thumb_distal_flex",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_distal,
        origin=Origin(xyz=(0.0, 0.0, thumb_proximal_spec["length"])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=1.15),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    palm = object_model.get_part("palm")
    thumb_base = object_model.get_part("thumb_base")
    index_distal = object_model.get_part("index_distal")
    thumb_distal = object_model.get_part("thumb_distal")

    expected_part_names = ["palm", "thumb_base", "thumb_proximal", "thumb_distal"]
    for finger_name in FINGER_NAMES:
        expected_part_names.extend(
            [f"{finger_name}_base", f"{finger_name}_proximal", f"{finger_name}_middle", f"{finger_name}_distal"]
        )

    expected_joint_axes = {"thumb_splay": (0.0, 1.0, 0.0), "thumb_knuckle_flex": (1.0, 0.0, 0.0), "thumb_distal_flex": (1.0, 0.0, 0.0)}
    for finger_name in FINGER_NAMES:
        expected_joint_axes[f"{finger_name}_splay"] = (0.0, 1.0, 0.0)
        expected_joint_axes[f"{finger_name}_knuckle_flex"] = (1.0, 0.0, 0.0)
        expected_joint_axes[f"{finger_name}_middle_flex"] = (1.0, 0.0, 0.0)
        expected_joint_axes[f"{finger_name}_distal_flex"] = (1.0, 0.0, 0.0)

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(palm, thumb_base, reason="Thumb base trunnion is seated into a palm-side metacarpal socket.")
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in expected_part_names:
        part = object_model.get_part(part_name)
        ctx.check(f"{part_name}_present", part is not None, details=f"Missing part {part_name}.")
        ctx.check(
            f"{part_name}_placed",
            ctx.part_world_aabb(part) is not None,
            details=f"No world AABB resolved for {part_name}.",
        )

    for joint_name, axis in expected_joint_axes.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_axis",
            tuple(joint.axis) == axis,
            details=f"{joint_name} axis {joint.axis} did not match expected {axis}.",
        )

    adjacency = [("thumb_base", "palm"), ("thumb_proximal", "thumb_base"), ("thumb_distal", "thumb_proximal")]
    for finger_name in FINGER_NAMES:
        adjacency.extend(
            [
                (f"{finger_name}_base", "palm"),
                (f"{finger_name}_proximal", f"{finger_name}_base"),
                (f"{finger_name}_middle", f"{finger_name}_proximal"),
                (f"{finger_name}_distal", f"{finger_name}_middle"),
            ]
        )
    for child_name, parent_name in adjacency:
        ctx.expect_contact(child_name, parent_name, contact_tol=0.0005, name=f"{child_name}_touches_{parent_name}")

    for finger_name in FINGER_NAMES:
        ctx.expect_overlap(f"{finger_name}_base", "palm", axes="xy", min_overlap=0.010, name=f"{finger_name}_base_over_palm")

    finger_base_centers = []
    for finger_name in FINGER_NAMES:
        aabb = ctx.part_world_aabb(object_model.get_part(f"{finger_name}_base"))
        assert aabb is not None
        finger_base_centers.append(_aabb_center(aabb)[0])
    ctx.check(
        "finger_roots_are_separate",
        all(b - a > 0.014 for a, b in zip(finger_base_centers, finger_base_centers[1:])),
        details=f"Finger root x-centers were too close: {finger_base_centers}",
    )

    index_rest = ctx.part_world_aabb(index_distal)
    thumb_rest = ctx.part_world_aabb(thumb_distal)
    assert index_rest is not None and thumb_rest is not None
    index_rest_center = _aabb_center(index_rest)
    thumb_rest_center = _aabb_center(thumb_rest)

    index_splay = object_model.get_articulation("index_splay")
    thumb_splay = object_model.get_articulation("thumb_splay")
    with ctx.pose({index_splay: -0.22, thumb_splay: 0.28}):
        index_spread = ctx.part_world_aabb(index_distal)
        thumb_spread = ctx.part_world_aabb(thumb_distal)
        assert index_spread is not None and thumb_spread is not None
        index_spread_center = _aabb_center(index_spread)
        thumb_spread_center = _aabb_center(thumb_spread)
        ctx.check(
            "index_splay_moves_laterally",
            index_spread_center[0] < index_rest_center[0] - 0.005,
            details=f"Index distal center did not swing laterally enough: rest {index_rest_center}, pose {index_spread_center}",
        )
        ctx.check(
            "thumb_splay_opposes_fingers",
            thumb_spread_center[0] > thumb_rest_center[0] + 0.004,
            details=f"Thumb distal center did not fan outward: rest {thumb_rest_center}, pose {thumb_spread_center}",
        )

    index_knuckle = object_model.get_articulation("index_knuckle_flex")
    index_middle = object_model.get_articulation("index_middle_flex")
    index_distal_joint = object_model.get_articulation("index_distal_flex")
    with ctx.pose({index_knuckle: 0.85, index_middle: 0.95, index_distal_joint: 0.70}):
        curled = ctx.part_world_aabb(index_distal)
        assert curled is not None
        curled_center = _aabb_center(curled)
        ctx.check(
            "index_flex_curls_downward",
            curled_center[1] < index_rest_center[1] - 0.012,
            details=f"Index distal center did not move toward the palm: rest {index_rest_center}, pose {curled_center}",
        )
        ctx.check(
            "index_flex_draws_tip_back",
            curled_center[2] < index_rest_center[2] - 0.020,
            details=f"Index distal center did not retract along z when flexed: rest {index_rest_center}, pose {curled_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
