from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _extruded_footprint(
    points: list[tuple[float, float]],
    *,
    height: float,
) -> MeshGeometry:
    """Simple low-profile keyboard-case shell from a 2D ergonomic footprint."""
    geom = MeshGeometry()
    lower = [geom.add_vertex(x, y, 0.0) for x, y in points]
    upper = [geom.add_vertex(x, y, height) for x, y in points]
    n = len(points)

    # Caps.  The outlines are authored counter-clockwise in local XY.
    for i in range(1, n - 1):
        geom.add_face(lower[0], lower[i + 1], lower[i])
        geom.add_face(upper[0], upper[i], upper[i + 1])

    # Side walls.
    for i in range(n):
        j = (i + 1) % n
        _add_quad(geom, lower[i], lower[j], upper[j], upper[i])
    return geom


def _half_transform(
    center: tuple[float, float, float],
    pitch: float,
    yaw: float,
    local: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Apply the same Rz(yaw) * Ry(pitch) transform used by Origin.rpy."""
    x, y, z = local
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    x1 = cp * x + sp * z
    y1 = y
    z1 = -sp * x + cp * z

    return (
        center[0] + cy * x1 - sy * y1,
        center[1] + sy * x1 + cy * y1,
        center[2] + z1,
    )


def _key_origin(
    center: tuple[float, float, float],
    pitch: float,
    yaw: float,
    x: float,
    y: float,
    z: float,
) -> Origin:
    return Origin(
        xyz=_half_transform(center, pitch, yaw, (x, y, z)),
        rpy=(0.0, pitch, yaw),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_ergonomic_keyboard")

    matte_black = model.material("matte_black", rgba=(0.055, 0.058, 0.062, 1.0))
    charcoal = model.material("charcoal_case", rgba=(0.135, 0.145, 0.155, 1.0))
    deck = model.material("black_key_plate", rgba=(0.025, 0.027, 0.030, 1.0))
    key_light = model.material("warm_keycaps", rgba=(0.83, 0.82, 0.77, 1.0))
    key_dark = model.material("dark_mod_keys", rgba=(0.17, 0.18, 0.19, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.035, 0.035, 0.035, 1.0))
    legend_blue = model.material("blue_thumb_keys", rgba=(0.22, 0.38, 0.62, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.47, 0.25, 0.075)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    # Each cluster is a wedge-like, chamfered slab.  The halves are yawed in a
    # shallow V and tented up at the outer edges; a short central bridge joins
    # the inner edges into one rigid split-keyboard body.
    half_outline = [
        (-0.130, -0.082),
        (0.092, -0.084),
        (0.128, -0.050),
        (0.130, 0.078),
        (0.040, 0.092),
        (-0.118, 0.080),
        (-0.136, 0.020),
    ]
    half_shell = mesh_from_geometry(
        _extruded_footprint(half_outline, height=0.024),
        "split_keyboard_half_shell",
    )

    # Object frame: +Y is toward the rear row, +Z is up.  The left/right labels
    # are intrinsic for a keyboard.
    left_center = (-0.167, 0.006, 0.020)
    right_center = (0.167, 0.006, 0.020)
    left_pitch = math.radians(9.0)
    right_pitch = math.radians(-9.0)
    left_yaw = math.radians(11.0)
    right_yaw = math.radians(-11.0)

    body.visual(
        half_shell,
        origin=Origin(xyz=left_center, rpy=(0.0, left_pitch, left_yaw)),
        material=charcoal,
        name="left_shell",
    )
    body.visual(
        half_shell,
        origin=Origin(xyz=right_center, rpy=(0.0, right_pitch, right_yaw)),
        material=charcoal,
        name="right_shell",
    )
    body.visual(
        Box((0.112, 0.066, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, 0.030)),
        material=charcoal,
        name="center_bridge",
    )
    body.visual(
        Box((0.082, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.012, 0.041)),
        material=matte_black,
        name="bridge_cover",
    )

    # Dark top plates and slim rear ledges make the key wells read as
    # manufactured recesses rather than keys floating above a plain slab.
    for side, center, pitch, yaw in (
        ("left", left_center, left_pitch, left_yaw),
        ("right", right_center, right_pitch, right_yaw),
    ):
        body.visual(
            Box((0.214, 0.132, 0.003)),
            origin=Origin(
                xyz=_half_transform(center, pitch, yaw, (-0.006, -0.003, 0.0255)),
                rpy=(0.0, pitch, yaw),
            ),
            material=deck,
            name=f"{side}_key_plate",
        )
        body.visual(
            Box((0.170, 0.012, 0.007)),
            origin=Origin(
                xyz=_half_transform(center, pitch, yaw, (-0.013, 0.078, 0.0265)),
                rpy=(0.0, pitch, yaw),
            ),
            material=matte_black,
            name=f"{side}_rear_lip",
        )
        body.visual(
            Box((0.020, 0.120, 0.006)),
            origin=Origin(
                xyz=_half_transform(
                    center,
                    pitch,
                    yaw,
                    ((0.106 if side == "left" else -0.106), -0.002, 0.027),
                ),
                rpy=(0.0, pitch, yaw),
            ),
            material=matte_black,
            name=f"{side}_inner_bezel",
        )
        body.visual(
            Box((0.150, 0.042, 0.003)),
            origin=Origin(
                xyz=_half_transform(
                    center,
                    pitch,
                    yaw,
                    ((0.045 if side == "left" else -0.045), -0.086, 0.0255),
                ),
                rpy=(0.0, pitch, yaw),
            ),
            material=deck,
            name=f"{side}_thumb_plate",
        )
        body.visual(
            Box((0.070, 0.014, 0.004)),
            origin=Origin(
                xyz=_half_transform(
                    center,
                    pitch,
                    yaw,
                    ((0.060 if side == "left" else -0.060), -0.082, 0.022),
                ),
                rpy=(0.0, pitch, yaw),
            ),
            material=matte_black,
            name=f"{side}_front_lip",
        )

    key_pitch_x = 0.038
    key_pitch_y = 0.032
    row_ys = (0.044, 0.012, -0.020, -0.052)
    key_z = 0.0265

    def add_key(
        *,
        side: str,
        center: tuple[float, float, float],
        pitch: float,
        yaw: float,
        name: str,
        x: float,
        y: float,
        key_material,
        size: tuple[float, float, float] = (0.030, 0.026, 0.007),
    ) -> None:
        key = model.part(name)
        key.visual(
            Box((size[0] * 0.74, size[1] * 0.56, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
            material=matte_black,
            name="switch_stem",
        )
        key.visual(
            Box(size),
            origin=Origin(xyz=(0.0, 0.0, 0.0100)),
            material=key_material,
            name="keycap",
        )
        key.inertial = Inertial.from_geometry(
            Box((size[0], size[1], 0.014)),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=_key_origin(center, pitch, yaw, x, y, key_z),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.10, lower=0.0, upper=0.004),
        )

    # Four staggered rows per half.  Outer columns are lower and wider where a
    # real ergonomic layout curves around the fingertips.
    for side, center, pitch, yaw, inner_sign in (
        ("left", left_center, left_pitch, left_yaw, 1.0),
        ("right", right_center, right_pitch, right_yaw, -1.0),
    ):
        for row_index, y in enumerate(row_ys):
            for col_index in range(5):
                x = (col_index - 2) * key_pitch_x
                y_stagger = y + (col_index - 2) * 0.003 * inner_sign + (row_index - 1.5) * 0.001
                material = key_dark if row_index == 0 and col_index in (0, 4) else key_light
                add_key(
                    side=side,
                    center=center,
                    pitch=pitch,
                    yaw=yaw,
                    name=f"{side}_key_{row_index}_{col_index}",
                    x=x,
                    y=y_stagger,
                    key_material=material,
                    size=(0.029, 0.025, 0.007),
                )

        # Three large thumb keys sweep inward at the front of each half.
        thumb_specs = (
            (0, 0.020 * inner_sign, -0.100, 0.037, 0.022),
            (1, 0.068 * inner_sign, -0.094, 0.040, 0.022),
            (2, 0.112 * inner_sign, -0.088, 0.032, 0.021),
        )
        for thumb_index, x, y, width, depth in thumb_specs:
            add_key(
                side=side,
                center=center,
                pitch=pitch,
                yaw=yaw,
                name=f"{side}_thumb_{thumb_index}",
                x=x,
                y=y,
                key_material=legend_blue if thumb_index == 1 else key_dark,
                size=(width, depth, 0.007),
            )

    def add_tenting_foot(
        *,
        side: str,
        center: tuple[float, float, float],
        pitch: float,
        yaw: float,
        x: float,
        y: float,
    ) -> None:
        foot = model.part(f"{side}_foot")
        foot.visual(
            Cylinder(radius=0.0055, length=0.070),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.056, 0.012, 0.078)),
            origin=Origin(xyz=(0.0, 0.000, -0.043)),
            material=rubber,
            name="folding_strut",
        )
        foot.visual(
            Box((0.074, 0.030, 0.008)),
            origin=Origin(xyz=(0.0, 0.002, -0.084)),
            material=rubber,
            name="desk_pad",
        )
        foot.inertial = Inertial.from_geometry(
            Box((0.080, 0.032, 0.090)),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, -0.045)),
        )
        model.articulation(
            f"body_to_{side}_foot",
            ArticulationType.REVOLUTE,
            parent=body,
            child=foot,
            origin=_key_origin(center, pitch, yaw, x, y, -0.0055),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=1.6, lower=0.0, upper=1.25),
        )

    # Rear outer fold-out feet; at rest they are deployed under each half,
    # holding the outer/rear shell high enough to clearly show the tent angle.
    add_tenting_foot(side="left", center=left_center, pitch=left_pitch, yaw=left_yaw, x=-0.090, y=0.066)
    add_tenting_foot(side="right", center=right_center, pitch=right_pitch, yaw=right_yaw, x=0.090, y=0.066)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    key_joints = [
        joint
        for joint in object_model.articulations
        if joint.name.startswith("body_to_left_key_")
        or joint.name.startswith("body_to_right_key_")
        or joint.name.startswith("body_to_left_thumb_")
        or joint.name.startswith("body_to_right_thumb_")
    ]
    ctx.check("individual_key_plungers", len(key_joints) == 46, f"found {len(key_joints)}")
    ctx.check(
        "key_travel_is_short",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.upper is not None
            and 0.003 <= joint.motion_limits.upper <= 0.005
            for joint in key_joints
        ),
        "Expected every key to have roughly 4 mm of prismatic travel.",
    )
    for joint in key_joints:
        key_name = joint.name.removeprefix("body_to_")
        side = "left" if key_name.startswith("left_") else "right"
        plate_elem = f"{side}_thumb_plate" if "_thumb_" in key_name else f"{side}_key_plate"
        ctx.allow_overlap(
            "body",
            key_name,
            elem_a=plate_elem,
            elem_b="switch_stem",
            reason=(
                "Each key's switch stem is intentionally seated in the key-plate "
                "socket proxy so the prismatic plunger reads mechanically captured."
            ),
        )
        ctx.expect_overlap(
            key_name,
            "body",
            axes="xy",
            elem_a="switch_stem",
            elem_b=plate_elem,
            min_overlap=0.010,
            name=f"{key_name}_stem_seated",
        )

    sample_key = object_model.get_part("left_key_1_2")
    sample_joint = object_model.get_articulation("body_to_left_key_1_2")
    rest_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_joint: 0.004}):
        pressed_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "sample_key_moves_down",
        rest_pos is not None
        and pressed_pos is not None
        and pressed_pos[2] < rest_pos[2] - 0.003,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    left_foot_joint = object_model.get_articulation("body_to_left_foot")
    right_foot_joint = object_model.get_articulation("body_to_right_foot")
    ctx.check("two_tenting_feet", left_foot is not None and right_foot is not None)
    ctx.check(
        "tenting_feet_revolute",
        left_foot_joint.articulation_type == ArticulationType.REVOLUTE
        and right_foot_joint.articulation_type == ArticulationType.REVOLUTE,
    )

    rest_left_aabb = ctx.part_world_aabb(left_foot)
    with ctx.pose({left_foot_joint: 1.25, right_foot_joint: 1.25}):
        folded_left_aabb = ctx.part_world_aabb(left_foot)
    ctx.check(
        "foot_folds_upward",
        rest_left_aabb is not None
        and folded_left_aabb is not None
        and folded_left_aabb[0][2] > rest_left_aabb[0][2] + 0.020,
        details=f"rest={rest_left_aabb}, folded={folded_left_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
