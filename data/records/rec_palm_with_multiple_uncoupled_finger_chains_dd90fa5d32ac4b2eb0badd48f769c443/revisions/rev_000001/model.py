from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    ValidationError,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


LONG_FINGER_SPECS = (
    {
        "name": "index",
        "root_xyz": (-0.031, 0.037, 0.023),
        "yaw": -0.13,
        "width": 0.0162,
        "height": 0.0120,
        "lengths": (0.041, 0.029, 0.022),
        "splay_limits": (-0.16, 0.38),
        "pose_splay": 0.22,
    },
    {
        "name": "middle",
        "root_xyz": (-0.010, 0.038, 0.023),
        "yaw": -0.02,
        "width": 0.0170,
        "height": 0.0124,
        "lengths": (0.045, 0.032, 0.024),
        "splay_limits": (-0.22, 0.22),
        "pose_splay": 0.10,
    },
    {
        "name": "ring",
        "root_xyz": (0.011, 0.037, 0.023),
        "yaw": 0.06,
        "width": 0.0161,
        "height": 0.0118,
        "lengths": (0.042, 0.030, 0.023),
        "splay_limits": (-0.24, 0.20),
        "pose_splay": -0.10,
    },
    {
        "name": "little",
        "root_xyz": (0.0335, 0.0345, 0.0225),
        "yaw": 0.22,
        "width": 0.0138,
        "height": 0.0108,
        "lengths": (0.033, 0.024, 0.019),
        "splay_limits": (-0.34, 0.12),
        "pose_splay": -0.22,
    },
)

THUMB_SPEC = {
    "name": "thumb",
    "root_xyz": (-0.047, -0.004, 0.022),
    "yaw": 0.98,
    "width": 0.0184,
    "height": 0.0132,
    "lengths": (0.029, 0.023),
    "splay_limits": (-0.45, 0.42),
}

LONG_FINGER_PARTS = tuple(
    part_name
    for spec in LONG_FINGER_SPECS
    for part_name in (
        f"{spec['name']}_base",
        f"{spec['name']}_proximal",
        f"{spec['name']}_middle",
        f"{spec['name']}_distal",
    )
)
THUMB_PARTS = ("thumb_base", "thumb_proximal", "thumb_distal")
EXPECTED_PARTS = ("palm",) + LONG_FINGER_PARTS + THUMB_PARTS

LONG_FINGER_JOINTS = tuple(
    joint_name
    for spec in LONG_FINGER_SPECS
    for joint_name in (
        f"{spec['name']}_splay",
        f"{spec['name']}_base_flex",
        f"{spec['name']}_middle_flex",
        f"{spec['name']}_tip_flex",
    )
)
THUMB_JOINTS = ("thumb_splay", "thumb_base_flex", "thumb_tip_flex")
EXPECTED_JOINTS = LONG_FINGER_JOINTS + THUMB_JOINTS


def _rounded_xz_section(
    width: float,
    height: float,
    y_pos: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    radius = min(width, height) * 0.26
    return [
        (x_pos, y_pos, z_pos + z_center)
        for x_pos, z_pos in rounded_rect_profile(
            width,
            height,
            radius=radius,
            corner_segments=6,
        )
    ]


def _build_palm_shell():
    sections = [
        _rounded_xz_section(0.074, 0.034, -0.043, z_center=0.000),
        _rounded_xz_section(0.088, 0.040, -0.014, z_center=0.002),
        _rounded_xz_section(0.092, 0.036, 0.018, z_center=0.003),
        _rounded_xz_section(0.080, 0.026, 0.041, z_center=0.002),
    ]
    return mesh_from_geometry(section_loft(sections), "robotic_palm_shell")


def _barrel_len(width: float) -> float:
    return width * 0.58


def _joint_radius(width: float) -> float:
    return max(0.0030, width * 0.23)


def _make_base_knuckle(
    part,
    *,
    shell_material,
    pin_material,
    width: float,
    height: float,
    flex_y: float,
    flex_z: float,
    child_barrel_len: float,
) -> None:
    turret_radius = width * 0.36
    turret_height = 0.009
    pivot_radius = max(0.0034, height * 0.33)
    inner_gap = child_barrel_len
    ear_t = max(0.0025, (width - inner_gap) * 0.5)
    ear_len = 0.012
    ear_height = height * 0.92
    ear_x = inner_gap * 0.5 + ear_t * 0.5

    part.visual(
        Cylinder(radius=turret_radius, length=turret_height),
        origin=Origin(xyz=(0.0, 0.0, turret_height * 0.5)),
        material=pin_material,
        name="root_turret",
    )
    part.visual(
        Cylinder(radius=width * 0.33, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, turret_height + 0.0015)),
        material=shell_material,
        name="root_collar",
    )
    part.visual(
        Box((width * 0.82, flex_y * 0.84, flex_z * 0.90)),
        origin=Origin(xyz=(0.0, flex_y * 0.34, flex_z * 0.46)),
        material=shell_material,
        name="core_block",
    )
    part.visual(
        Box((width * 0.58, flex_y * 0.54, flex_z * 0.40)),
        origin=Origin(xyz=(0.0, flex_y * 0.42, flex_z * 0.76)),
        material=shell_material,
        name="dorsal_cap",
    )
    part.visual(
        Box((width * 0.54, 0.0045, pivot_radius * 1.7)),
        origin=Origin(xyz=(0.0, flex_y - 0.0075, flex_z)),
        material=pin_material,
        name="front_brace",
    )
    for sign in (-1.0, 1.0):
        part.visual(
            Box((ear_t, ear_len, ear_height)),
            origin=Origin(xyz=(sign * ear_x, flex_y - ear_len * 0.48, flex_z)),
            material=shell_material,
            name=f"ear_{'l' if sign < 0.0 else 'r'}",
        )
        part.visual(
            Cylinder(radius=pivot_radius * 0.76, length=0.0028),
            origin=Origin(
                xyz=(sign * (ear_x + ear_t * 0.5 - 0.0007), flex_y, flex_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=pin_material,
            name=f"pin_cap_{'l' if sign < 0.0 else 'r'}",
        )
    part.inertial = Inertial.from_geometry(
        Box((width, flex_y + 0.008, flex_z + 0.012)),
        mass=0.045,
        origin=Origin(xyz=(0.0, flex_y * 0.35, (flex_z + 0.012) * 0.42)),
    )


def _make_finger_segment(
    part,
    *,
    shell_material,
    pin_material,
    pad_material,
    width: float,
    height: float,
    span: float,
    barrel_len: float,
    child_barrel_len: float | None,
    tip: bool,
) -> None:
    joint_radius = _joint_radius(width)
    neck_len = min(0.006, span * 0.20)
    shell_stop = 0.006 if child_barrel_len is not None else 0.001
    shell_end = max(span - shell_stop, neck_len + 0.006)
    beam_start = neck_len * 0.70
    beam_len = max(shell_end - beam_start, span * 0.48)

    part.visual(
        Cylinder(radius=joint_radius, length=barrel_len),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=pin_material,
        name="rear_barrel",
    )
    part.visual(
        Box((barrel_len * 0.82, neck_len, height * 0.58)),
        origin=Origin(xyz=(0.0, neck_len * 0.5, 0.0)),
        material=shell_material,
        name="hinge_neck",
    )
    part.visual(
        Box((width * 0.72, beam_len, height * 0.80)),
        origin=Origin(xyz=(0.0, beam_start + beam_len * 0.5, 0.0)),
        material=shell_material,
        name="main_beam",
    )
    part.visual(
        Box((width * 0.50, beam_len * 0.72, height * 0.46)),
        origin=Origin(
            xyz=(0.0, beam_start + beam_len * 0.46, height * 0.22),
        ),
        material=shell_material,
        name="dorsal_cap",
    )
    part.visual(
        Box((width * 0.28, beam_len * 0.62, height * 0.18)),
        origin=Origin(
            xyz=(0.0, beam_start + beam_len * 0.48, -height * 0.24),
        ),
        material=pin_material,
        name="underside_rib",
    )

    if child_barrel_len is not None:
        inner_gap = child_barrel_len
        ear_t = max(0.0022, (width - inner_gap) * 0.5)
        ear_len = min(max(0.010, span * 0.24), span * 0.55)
        ear_height = height * 0.94
        ear_x = inner_gap * 0.5 + ear_t * 0.5
        part.visual(
            Box((width * 0.54, max(0.0035, ear_len * 0.35), height * 0.40)),
            origin=Origin(xyz=(0.0, span - ear_len * 0.82, 0.0)),
            material=pin_material,
            name="front_brace",
        )
        for sign in (-1.0, 1.0):
            part.visual(
                Box((ear_t, ear_len, ear_height)),
                origin=Origin(xyz=(sign * ear_x, span - ear_len * 0.48, 0.0)),
                material=shell_material,
                name=f"ear_{'l' if sign < 0.0 else 'r'}",
            )

    if tip:
        pad_radius = max(0.0038, height * 0.42)
        part.visual(
            Sphere(radius=pad_radius),
            origin=Origin(xyz=(0.0, span + pad_radius * 0.28, -height * 0.04)),
            material=pad_material,
            name="tip_pad",
        )
        part.visual(
            Box((width * 0.60, pad_radius * 1.25, height * 0.22)),
            origin=Origin(
                xyz=(0.0, span - pad_radius * 0.05, -height * 0.24),
            ),
            material=pad_material,
            name="pad_plate",
        )
        inertial_len = span + pad_radius * 0.9
        mass = 0.020
    else:
        inertial_len = span
        mass = 0.028

    part.inertial = Inertial.from_geometry(
        Box((width, inertial_len, height * 1.15)),
        mass=mass,
        origin=Origin(xyz=(0.0, inertial_len * 0.5, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anthropomorphic_robotic_palm")

    palm_shell = model.material("palm_shell", rgba=(0.24, 0.27, 0.30, 1.0))
    knuckle_shell = model.material("knuckle_shell", rgba=(0.57, 0.60, 0.64, 1.0))
    joint_metal = model.material("joint_metal", rgba=(0.30, 0.33, 0.36, 1.0))
    fingertip_pad = model.material("fingertip_pad", rgba=(0.09, 0.10, 0.11, 1.0))

    palm = model.part("palm")
    palm.visual(_build_palm_shell(), material=palm_shell, name="palm_shell")
    palm.visual(
        Box((0.060, 0.015, 0.012)),
        origin=Origin(xyz=(0.0, -0.048, -0.002)),
        material=joint_metal,
        name="wrist_stub",
    )
    palm.visual(
        Box((0.070, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.031, 0.016)),
        material=knuckle_shell,
        name="finger_deck",
    )
    palm.visual(
        Box((0.030, 0.031, 0.014)),
        origin=Origin(xyz=(-0.041, -0.004, 0.009), rpy=(0.0, 0.0, 0.52)),
        material=knuckle_shell,
        name="thumb_buttress",
    )
    palm.visual(
        Box((0.020, 0.022, 0.010)),
        origin=Origin(xyz=(-0.031, 0.016, 0.014)),
        material=knuckle_shell,
        name="thenar_ridge",
    )
    for spec in LONG_FINGER_SPECS:
        boss_w = spec["width"] * 1.24
        palm.visual(
            Box((boss_w, 0.012, 0.008)),
            origin=Origin(
                xyz=(
                    spec["root_xyz"][0],
                    spec["root_xyz"][1] - 0.004,
                    spec["root_xyz"][2] - 0.004,
                )
            ),
            material=knuckle_shell,
            name=f"{spec['name']}_boss",
        )
    palm.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(
            xyz=(
                THUMB_SPEC["root_xyz"][0],
                THUMB_SPEC["root_xyz"][1],
                THUMB_SPEC["root_xyz"][2] - 0.004,
            )
        ),
        material=knuckle_shell,
        name="thumb_boss",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.104, 0.094, 0.050)),
        mass=0.68,
        origin=Origin(xyz=(0.0, -0.002, 0.003)),
    )

    for spec in LONG_FINGER_SPECS:
        name = spec["name"]
        width = spec["width"]
        height = spec["height"]
        proximal_len, middle_len, distal_len = spec["lengths"]

        mid_width = width * 0.92
        mid_height = height * 0.92
        dist_width = width * 0.84
        dist_height = height * 0.84

        base_part = model.part(f"{name}_base")
        _make_base_knuckle(
            base_part,
            shell_material=knuckle_shell,
            pin_material=joint_metal,
            width=width * 1.10,
            height=height * 1.02,
            flex_y=0.016,
            flex_z=height * 0.88,
            child_barrel_len=_barrel_len(width),
        )

        proximal = model.part(f"{name}_proximal")
        _make_finger_segment(
            proximal,
            shell_material=knuckle_shell,
            pin_material=joint_metal,
            pad_material=fingertip_pad,
            width=width,
            height=height,
            span=proximal_len,
            barrel_len=_barrel_len(width),
            child_barrel_len=_barrel_len(mid_width),
            tip=False,
        )

        middle = model.part(f"{name}_middle")
        _make_finger_segment(
            middle,
            shell_material=knuckle_shell,
            pin_material=joint_metal,
            pad_material=fingertip_pad,
            width=mid_width,
            height=mid_height,
            span=middle_len,
            barrel_len=_barrel_len(mid_width),
            child_barrel_len=_barrel_len(dist_width),
            tip=False,
        )

        distal = model.part(f"{name}_distal")
        _make_finger_segment(
            distal,
            shell_material=knuckle_shell,
            pin_material=joint_metal,
            pad_material=fingertip_pad,
            width=dist_width,
            height=dist_height,
            span=distal_len,
            barrel_len=_barrel_len(dist_width),
            child_barrel_len=None,
            tip=True,
        )

        model.articulation(
            f"{name}_splay",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=base_part,
            origin=Origin(xyz=spec["root_xyz"], rpy=(0.0, 0.0, spec["yaw"])),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=2.0,
                lower=spec["splay_limits"][0],
                upper=spec["splay_limits"][1],
            ),
        )
        model.articulation(
            f"{name}_base_flex",
            ArticulationType.REVOLUTE,
            parent=base_part,
            child=proximal,
            origin=Origin(xyz=(0.0, 0.016, height * 0.88)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.2,
                velocity=2.8,
                lower=0.0,
                upper=1.45,
            ),
        )
        model.articulation(
            f"{name}_middle_flex",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, proximal_len, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.7,
                velocity=3.0,
                lower=0.0,
                upper=1.55,
            ),
        )
        model.articulation(
            f"{name}_tip_flex",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, middle_len, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.1,
                velocity=3.0,
                lower=0.0,
                upper=1.25,
            ),
        )

    thumb_width = THUMB_SPEC["width"]
    thumb_height = THUMB_SPEC["height"]
    thumb_prox_len, thumb_dist_len = THUMB_SPEC["lengths"]
    thumb_dist_width = thumb_width * 0.84
    thumb_dist_height = thumb_height * 0.86

    thumb_base = model.part("thumb_base")
    _make_base_knuckle(
        thumb_base,
        shell_material=knuckle_shell,
        pin_material=joint_metal,
        width=thumb_width * 1.12,
        height=thumb_height * 1.04,
        flex_y=0.015,
        flex_z=thumb_height * 0.82,
        child_barrel_len=_barrel_len(thumb_width),
    )

    thumb_proximal = model.part("thumb_proximal")
    _make_finger_segment(
        thumb_proximal,
        shell_material=knuckle_shell,
        pin_material=joint_metal,
        pad_material=fingertip_pad,
        width=thumb_width,
        height=thumb_height,
        span=thumb_prox_len,
        barrel_len=_barrel_len(thumb_width),
        child_barrel_len=_barrel_len(thumb_dist_width),
        tip=False,
    )

    thumb_distal = model.part("thumb_distal")
    _make_finger_segment(
        thumb_distal,
        shell_material=knuckle_shell,
        pin_material=joint_metal,
        pad_material=fingertip_pad,
        width=thumb_dist_width,
        height=thumb_dist_height,
        span=thumb_dist_len,
        barrel_len=_barrel_len(thumb_dist_width),
        child_barrel_len=None,
        tip=True,
    )

    model.articulation(
        "thumb_splay",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_base,
        origin=Origin(xyz=THUMB_SPEC["root_xyz"], rpy=(0.0, 0.0, THUMB_SPEC["yaw"])),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.2,
            lower=THUMB_SPEC["splay_limits"][0],
            upper=THUMB_SPEC["splay_limits"][1],
        ),
    )
    model.articulation(
        "thumb_base_flex",
        ArticulationType.REVOLUTE,
        parent=thumb_base,
        child=thumb_proximal,
        origin=Origin(xyz=(0.0, 0.015, thumb_height * 0.82)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.6,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "thumb_tip_flex",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_distal,
        origin=Origin(xyz=(0.0, thumb_prox_len, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.3,
            velocity=2.8,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def _expect_lookup(ctx: TestContext, getter, item_name: str, *, kind: str) -> None:
    try:
        getter(item_name)
    except ValidationError as exc:
        ctx.fail(f"has_{item_name}", f"Missing {kind}: {exc}")
    else:
        ctx.check(f"has_{item_name}", True)


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    palm = object_model.get_part("palm")
    _expect_lookup(ctx, object_model.get_part, "palm", kind="part")
    for part_name in EXPECTED_PARTS[1:]:
        _expect_lookup(ctx, object_model.get_part, part_name, kind="part")
    for joint_name in EXPECTED_JOINTS:
        _expect_lookup(ctx, object_model.get_articulation, joint_name, kind="articulation")

    for spec in LONG_FINGER_SPECS:
        name = spec["name"]
        base = object_model.get_part(f"{name}_base")
        proximal = object_model.get_part(f"{name}_proximal")
        middle = object_model.get_part(f"{name}_middle")
        distal = object_model.get_part(f"{name}_distal")

        ctx.expect_contact(
            palm,
            base,
            contact_tol=0.0005,
            name=f"{name}_base_mount_contact",
        )
        ctx.expect_contact(
            base,
            proximal,
            contact_tol=0.0005,
            name=f"{name}_base_knuckle_contact",
        )
        ctx.expect_contact(
            proximal,
            middle,
            contact_tol=0.0005,
            name=f"{name}_middle_knuckle_contact",
        )
        ctx.expect_contact(
            middle,
            distal,
            contact_tol=0.0005,
            name=f"{name}_distal_knuckle_contact",
        )

        splay = object_model.get_articulation(f"{name}_splay")
        base_flex = object_model.get_articulation(f"{name}_base_flex")
        middle_flex = object_model.get_articulation(f"{name}_middle_flex")
        tip_flex = object_model.get_articulation(f"{name}_tip_flex")

        ctx.check(
            f"{name}_splay_axis_is_z",
            tuple(splay.axis) == (0.0, 0.0, 1.0),
            f"Expected {name} splay axis to be local z, got {splay.axis}.",
        )
        ctx.check(
            f"{name}_flex_axes_are_x",
            tuple(base_flex.axis) == (-1.0, 0.0, 0.0)
            and tuple(middle_flex.axis) == (-1.0, 0.0, 0.0)
            and tuple(tip_flex.axis) == (-1.0, 0.0, 0.0),
            "Finger flex joints should all hinge about local x.",
        )

        pose_map = {
            splay: spec["pose_splay"],
            base_flex: 0.62,
            middle_flex: 0.88,
            tip_flex: 0.56,
        }
        with ctx.pose(pose_map):
            ctx.expect_contact(
                palm,
                base,
                contact_tol=0.0005,
                name=f"{name}_splayed_mount_stays_captured",
            )
            ctx.expect_contact(
                base,
                proximal,
                contact_tol=0.0005,
                name=f"{name}_base_joint_stays_captured",
            )
            ctx.expect_contact(
                proximal,
                middle,
                contact_tol=0.0005,
                name=f"{name}_middle_joint_stays_captured",
            )
            ctx.expect_contact(
                middle,
                distal,
                contact_tol=0.0005,
                name=f"{name}_tip_joint_stays_captured",
            )

    thumb_base = object_model.get_part("thumb_base")
    thumb_proximal = object_model.get_part("thumb_proximal")
    thumb_distal = object_model.get_part("thumb_distal")
    thumb_splay = object_model.get_articulation("thumb_splay")
    thumb_base_flex = object_model.get_articulation("thumb_base_flex")
    thumb_tip_flex = object_model.get_articulation("thumb_tip_flex")

    ctx.expect_contact(
        palm,
        thumb_base,
        contact_tol=0.0005,
        name="thumb_base_mount_contact",
    )
    ctx.expect_contact(
        thumb_base,
        thumb_proximal,
        contact_tol=0.0005,
        name="thumb_base_knuckle_contact",
    )
    ctx.expect_contact(
        thumb_proximal,
        thumb_distal,
        contact_tol=0.0005,
        name="thumb_tip_knuckle_contact",
    )
    ctx.check(
        "thumb_splay_axis_is_z",
        tuple(thumb_splay.axis) == (0.0, 0.0, 1.0),
        f"Expected thumb splay axis to be local z, got {thumb_splay.axis}.",
    )
    ctx.check(
        "thumb_flex_axes_are_x",
        tuple(thumb_base_flex.axis) == (-1.0, 0.0, 0.0)
        and tuple(thumb_tip_flex.axis) == (-1.0, 0.0, 0.0),
        "Thumb flex joints should hinge about local x.",
    )

    def tip_center(part_name: str) -> tuple[float, float, float]:
        tip_aabb = ctx.part_element_world_aabb(part_name, elem="tip_pad")
        assert tip_aabb is not None
        return _aabb_center(tip_aabb)

    index_tip_rest = tip_center("index_distal")
    with ctx.pose({thumb_splay: 0.18, thumb_base_flex: 0.70, thumb_tip_flex: 0.55}):
        thumb_tip_curled = tip_center("thumb_distal")
        ctx.expect_contact(
            palm,
            thumb_base,
            contact_tol=0.0005,
            name="thumb_mount_stays_captured_in_pose",
        )
        ctx.expect_contact(
            thumb_base,
            thumb_proximal,
            contact_tol=0.0005,
            name="thumb_base_joint_stays_captured",
        )
        ctx.expect_contact(
            thumb_proximal,
            thumb_distal,
            contact_tol=0.0005,
            name="thumb_tip_joint_stays_captured",
        )
        ctx.check(
            "thumb_curl_moves_tip_below_index_tip_height",
            thumb_tip_curled[2] < index_tip_rest[2] + 0.005,
            "Thumb tip should curl toward the palm plane.",
        )

    with ctx.pose(index_splay=0.26):
        index_tip_splayed = tip_center("index_distal")
        ctx.check(
            "index_splay_moves_tip_sideways",
            index_tip_splayed[0] < index_tip_rest[0] - 0.004,
            "Index fingertip should move outward during splay.",
        )

    with ctx.pose(index_base_flex=0.82, index_middle_flex=0.92, index_tip_flex=0.64):
        index_tip_curled = tip_center("index_distal")
        ctx.check(
            "index_flex_curls_tip_downward",
            index_tip_curled[2] < index_tip_rest[2] - 0.012,
            "Index fingertip should curl downward under flexion.",
        )

    rest_tip_centers = [tip_center(f"{spec['name']}_distal") for spec in LONG_FINGER_SPECS]
    ctx.check(
        "long_fingertips_ordered_across_palm",
        rest_tip_centers[0][0] < rest_tip_centers[1][0] < rest_tip_centers[2][0] < rest_tip_centers[3][0],
        "Index through little fingertips should remain laterally ordered.",
    )
    ctx.check(
        "middle_finger_is_most_forward",
        rest_tip_centers[1][1] > rest_tip_centers[0][1]
        and rest_tip_centers[1][1] > rest_tip_centers[2][1]
        and rest_tip_centers[1][1] > rest_tip_centers[3][1],
        "Middle finger should project the farthest from the palm.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
