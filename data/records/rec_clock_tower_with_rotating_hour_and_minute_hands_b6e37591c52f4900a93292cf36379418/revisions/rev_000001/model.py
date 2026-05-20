from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rpy_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return URDF-compatible rpy for a rotation matrix defined by local axes."""

    r00, r10, r20 = x_axis
    r01, r11, r21 = y_axis
    r02, r12, r22 = z_axis
    _ = (r01, r02, r11, r12)  # retained for readability of the matrix layout
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    cp = math.cos(pitch)
    if abs(cp) > 1.0e-8:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:
        roll = 0.0
        yaw = math.atan2(-r01, r11)
    return (roll, pitch, yaw)


def _build_pyramidal_roof_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    half = 0.82
    eave = 0.10
    apex_z = 0.92
    base_z = 0.00
    ids = [
        geom.add_vertex(-half - eave, -half - eave, base_z),
        geom.add_vertex(half + eave, -half - eave, base_z),
        geom.add_vertex(half + eave, half + eave, base_z),
        geom.add_vertex(-half - eave, half + eave, base_z),
        geom.add_vertex(0.0, 0.0, apex_z),
    ]
    # Four slate roof planes plus a hidden square underside to make a closed mesh.
    geom.add_face(ids[0], ids[1], ids[4])
    geom.add_face(ids[1], ids[2], ids[4])
    geom.add_face(ids[2], ids[3], ids[4])
    geom.add_face(ids[3], ids[0], ids[4])
    geom.add_face(ids[0], ids[3], ids[2])
    geom.add_face(ids[0], ids[2], ids[1])
    return geom


FACE_SPECS: dict[str, dict[str, tuple[float, float, float] | str]] = {
    "front": {
        "normal": (0.0, 1.0, 0.0),
        "u": (-1.0, 0.0, 0.0),
        "v": (0.0, 0.0, 1.0),
    },
    "rear": {
        "normal": (0.0, -1.0, 0.0),
        "u": (1.0, 0.0, 0.0),
        "v": (0.0, 0.0, 1.0),
    },
    "side_0": {
        "normal": (1.0, 0.0, 0.0),
        "u": (0.0, 1.0, 0.0),
        "v": (0.0, 0.0, 1.0),
    },
    "side_1": {
        "normal": (-1.0, 0.0, 0.0),
        "u": (0.0, -1.0, 0.0),
        "v": (0.0, 0.0, 1.0),
    },
}


def _face_point(
    face: str,
    *,
    u_coord: float = 0.0,
    v_coord: float = 0.0,
    outward: float = 0.0,
    z_center: float = 0.0,
) -> tuple[float, float, float]:
    spec = FACE_SPECS[face]
    normal = spec["normal"]  # type: ignore[assignment]
    u_axis = spec["u"]  # type: ignore[assignment]
    v_axis = spec["v"]  # type: ignore[assignment]
    return (
        u_axis[0] * u_coord + v_axis[0] * v_coord + normal[0] * outward,
        u_axis[1] * u_coord + v_axis[1] * v_coord + normal[1] * outward,
        z_center + u_axis[2] * u_coord + v_axis[2] * v_coord + normal[2] * outward,
    )


def _face_rpy(face: str) -> tuple[float, float, float]:
    spec = FACE_SPECS[face]
    return _rpy_from_axes(
        spec["u"],  # type: ignore[arg-type]
        spec["v"],  # type: ignore[arg-type]
        spec["normal"],  # type: ignore[arg-type]
    )


def _add_clock_face(
    tower,
    face: str,
    *,
    z_center: float,
    stage_half: float,
    materials: dict[str, object],
) -> None:
    rpy = _face_rpy(face)
    normal_out = stage_half

    tower.visual(
        Box((0.96, 0.96, 0.045)),
        origin=Origin(xyz=_face_point(face, outward=normal_out + 0.018, z_center=z_center), rpy=rpy),
        material=materials["stone_shadow"],
        name=f"{face}_recess_panel",
    )
    tower.visual(
        Cylinder(radius=0.505, length=0.038),
        origin=Origin(xyz=_face_point(face, outward=normal_out + 0.040, z_center=z_center), rpy=rpy),
        material=materials["blackened_bronze"],
        name=f"{face}_clock_rim",
    )
    tower.visual(
        Cylinder(radius=0.455, length=0.045),
        origin=Origin(xyz=_face_point(face, outward=normal_out + 0.061, z_center=z_center), rpy=rpy),
        material=materials["aged_ivory"],
        name=f"{face}_dial",
    )

    for index in range(12):
        angle = 2.0 * math.pi * index / 12.0
        u_coord = 0.365 * math.sin(angle)
        v_coord = 0.365 * math.cos(angle)
        dot_radius = 0.022 if index % 3 else 0.032
        tower.visual(
            Cylinder(radius=dot_radius, length=0.014),
            origin=Origin(
                xyz=_face_point(
                    face,
                    u_coord=u_coord,
                    v_coord=v_coord,
                    outward=normal_out + 0.0885,
                    z_center=z_center,
                ),
                rpy=rpy,
            ),
            material=materials["blackened_bronze"],
            name=f"{face}_hour_mark_{index}",
        )


def _add_hand_part(
    model: ArticulatedObject,
    tower,
    face: str,
    hand_kind: str,
    *,
    z_center: float,
    stage_half: float,
    materials: dict[str, object],
):
    is_minute = hand_kind == "minute"
    length = 0.390 if is_minute else 0.285
    width = 0.030 if is_minute else 0.046
    counter = 0.105 if is_minute else 0.070
    blade_root = 0.022
    blade_span = length - blade_root
    counter_span = counter - blade_root
    normal_out = stage_half + (0.154 if is_minute else 0.112)
    dial_front_out = stage_half + 0.061 + 0.045 * 0.5
    arbor_rear = dial_front_out - 0.003 - normal_out
    arbor_front = 0.006
    arbor_length = arbor_front - arbor_rear
    part_name = f"{face}_{hand_kind}_hand"
    joint_name = f"{face}_{hand_kind}_pivot"

    hand = model.part(part_name)
    if is_minute:
        hand.visual(
            Box((width, blade_span, 0.016)),
            origin=Origin(xyz=(0.0, blade_root + blade_span * 0.5, 0.0)),
            material=materials["blackened_bronze"],
            name="minute_blade",
        )
        hand.visual(
            Box((width * 0.62, counter_span, 0.014)),
            origin=Origin(xyz=(0.0, -(blade_root + counter_span * 0.5), 0.0)),
            material=materials["blackened_bronze"],
            name="minute_counterweight",
        )
        hand.visual(
            Cylinder(radius=0.050, length=0.020),
            material=materials["blackened_bronze"],
            name="minute_boss",
        )
        hand.visual(
            Cylinder(radius=0.012, length=arbor_length),
            origin=Origin(xyz=(0.0, 0.0, (arbor_front + arbor_rear) * 0.5)),
            material=materials["blackened_bronze"],
            name="minute_arbor",
        )
    else:
        hand.visual(
            Box((width, blade_span, 0.016)),
            origin=Origin(xyz=(0.0, blade_root + blade_span * 0.5, 0.0)),
            material=materials["blackened_bronze"],
            name="hour_blade",
        )
        hand.visual(
            Box((width * 0.62, counter_span, 0.014)),
            origin=Origin(xyz=(0.0, -(blade_root + counter_span * 0.5), 0.0)),
            material=materials["blackened_bronze"],
            name="hour_counterweight",
        )
        hand.visual(
            Cylinder(radius=0.044, length=0.020),
            material=materials["blackened_bronze"],
            name="hour_boss",
        )
        hand.visual(
            Cylinder(radius=0.016, length=arbor_length),
            origin=Origin(xyz=(0.0, 0.0, (arbor_front + arbor_rear) * 0.5)),
            material=materials["blackened_bronze"],
            name="hour_arbor",
        )

    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hand,
        origin=Origin(xyz=_face_point(face, outward=normal_out, z_center=z_center), rpy=_face_rpy(face)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25 if is_minute else 0.35,
            velocity=0.45 if is_minute else 0.08,
            lower=0.0,
            upper=2.0 * math.pi,
        ),
    )
    return hand


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="victorian_civic_clock_tower")

    materials = {
        "stone": model.material("warm_limestone", rgba=(0.66, 0.62, 0.53, 1.0)),
        "stone_light": model.material("pale_stone_edges", rgba=(0.76, 0.73, 0.64, 1.0)),
        "stone_shadow": model.material("recess_shadow_stone", rgba=(0.36, 0.34, 0.31, 1.0)),
        "aged_ivory": model.material("aged_ivory_enamel", rgba=(0.92, 0.88, 0.72, 1.0)),
        "blackened_bronze": model.material("blackened_bronze", rgba=(0.03, 0.025, 0.018, 1.0)),
        "slate": model.material("blue_grey_slate", rgba=(0.16, 0.19, 0.22, 1.0)),
        "glass": model.material("dark_glass", rgba=(0.05, 0.07, 0.08, 1.0)),
    }

    tower = model.part("tower")
    tower.visual(
        Box((2.15, 2.15, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=materials["stone_shadow"],
        name="ground_plinth",
    )
    tower.visual(
        Box((1.82, 1.82, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=materials["stone"],
        name="stepped_plinth",
    )
    tower.visual(
        Box((1.24, 1.24, 4.80)),
        origin=Origin(xyz=(0.0, 0.0, 2.86)),
        material=materials["stone"],
        name="square_shaft",
    )

    # Dressed corner quoins and shallow horizontal courses make the shaft read as masonry.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((0.19, 0.19, 4.92)),
                origin=Origin(xyz=(sx * 0.545, sy * 0.545, 2.86)),
                material=materials["stone_light"],
                name=f"quoin_{sx:g}_{sy:g}",
            )

    for index, z in enumerate((0.84, 1.48, 2.12, 2.76, 3.40, 4.04, 4.68)):
        tower.visual(
            Box((1.34, 1.34, 0.050)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=materials["stone_light"],
            name=f"masonry_course_{index}",
        )

    # Civic entrance and narrow upper lancet windows.
    tower.visual(
        Box((0.44, 0.040, 0.82)),
        origin=Origin(xyz=(0.0, 0.642, 0.93)),
        material=materials["blackened_bronze"],
        name="front_door",
    )
    tower.visual(
        Box((0.58, 0.070, 0.15)),
        origin=Origin(xyz=(0.0, 0.660, 1.40)),
        material=materials["stone_light"],
        name="door_lintel",
    )
    for face, x, y, rpy in (
        ("front_window", 0.0, 0.642, (0.0, 0.0, 0.0)),
        ("rear_window", 0.0, -0.642, (0.0, 0.0, 0.0)),
    ):
        tower.visual(
            Box((0.30, 0.038, 0.72)),
            origin=Origin(xyz=(x, y, 3.35), rpy=rpy),
            material=materials["glass"],
            name=face,
        )
    for name, x, y in (("side_window_0", 0.642, 0.0), ("side_window_1", -0.642, 0.0)):
        tower.visual(
            Box((0.038, 0.30, 0.72)),
            origin=Origin(xyz=(x, y, 3.35)),
            material=materials["glass"],
            name=name,
        )

    # Corbelled transition: a neck band, individual projecting blocks, and the wider clock stage.
    tower.visual(
        Box((1.38, 1.38, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 5.30)),
        material=materials["stone_light"],
        name="neck_band",
    )
    for side in ("front", "rear"):
        y = 0.705 if side == "front" else -0.705
        for idx, x in enumerate((-0.42, 0.0, 0.42)):
            tower.visual(
                Box((0.20, 0.30, 0.30)),
                origin=Origin(xyz=(x, y, 5.45)),
                material=materials["stone_light"],
                name=f"{side}_corbel_{idx}",
            )
    for side in ("side_0", "side_1"):
        x = 0.705 if side == "side_0" else -0.705
        for idx, y in enumerate((-0.42, 0.0, 0.42)):
            tower.visual(
                Box((0.30, 0.20, 0.30)),
                origin=Origin(xyz=(x, y, 5.45)),
                material=materials["stone_light"],
                name=f"{side}_corbel_{idx}",
            )
    tower.visual(
        Box((1.58, 1.58, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 5.55)),
        material=materials["stone_light"],
        name="corbel_slab",
    )
    tower.visual(
        Box((1.72, 1.72, 1.52)),
        origin=Origin(xyz=(0.0, 0.0, 6.36)),
        material=materials["stone"],
        name="clock_stage",
    )
    tower.visual(
        Box((1.90, 1.90, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 7.22)),
        material=materials["stone_light"],
        name="upper_cornice",
    )
    tower.visual(
        mesh_from_geometry(_build_pyramidal_roof_mesh(), "clock_tower_pyramidal_slate_roof"),
        origin=Origin(xyz=(0.0, 0.0, 7.27)),
        material=materials["slate"],
        name="slate_roof",
    )
    tower.visual(
        Cylinder(radius=0.045, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 8.46)),
        material=materials["blackened_bronze"],
        name="roof_finial",
    )
    tower.visual(
        Sphere(radius=0.075),
        origin=Origin(xyz=(0.0, 0.0, 8.76)),
        material=materials["blackened_bronze"],
        name="finial_ball",
    )

    clock_z = 6.42
    stage_half = 0.86
    for face in ("front", "rear", "side_0", "side_1"):
        _add_clock_face(tower, face, z_center=clock_z, stage_half=stage_half, materials=materials)
        _add_hand_part(
            model,
            tower,
            face,
            "hour",
            z_center=clock_z,
            stage_half=stage_half,
            materials=materials,
        )
        _add_hand_part(
            model,
            tower,
            face,
            "minute",
            z_center=clock_z,
            stage_half=stage_half,
            materials=materials,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")

    hand_joint_names = [
        f"{face}_{kind}_pivot"
        for face in ("front", "rear", "side_0", "side_1")
        for kind in ("hour", "minute")
    ]
    ctx.check(
        "four faces each carry hour and minute revolute pivots",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE
            for name in hand_joint_names
        ),
        details=f"hand joints={hand_joint_names}",
    )

    # Each set of hands is layered proud of its dial instead of intersecting it.
    face_gap_specs = (
        ("front", "y", False),
        ("rear", "y", True),
        ("side_0", "x", False),
        ("side_1", "x", True),
    )
    for face, axis, tower_is_positive in face_gap_specs:
        for kind in ("hour", "minute"):
            hand = object_model.get_part(f"{face}_{kind}_hand")
            ctx.allow_overlap(
                hand,
                tower,
                elem_a=f"{kind}_arbor",
                elem_b=f"{face}_dial",
                reason="The hand arbor is intentionally seated a few millimetres into the clock dial bushing.",
            )
            if tower_is_positive:
                ctx.expect_gap(
                    tower,
                    hand,
                    axis=axis,
                    min_gap=0.008,
                    positive_elem=f"{face}_dial",
                    negative_elem=f"{kind}_blade",
                    name=f"{face} {kind} hand is proud of dial",
                )
                ctx.expect_gap(
                    tower,
                    hand,
                    axis=axis,
                    max_penetration=0.004,
                    positive_elem=f"{face}_dial",
                    negative_elem=f"{kind}_arbor",
                    name=f"{face} {kind} arbor has shallow dial insertion",
                )
            else:
                ctx.expect_gap(
                    hand,
                    tower,
                    axis=axis,
                    min_gap=0.008,
                    positive_elem=f"{kind}_blade",
                    negative_elem=f"{face}_dial",
                    name=f"{face} {kind} hand is proud of dial",
                )
                ctx.expect_gap(
                    hand,
                    tower,
                    axis=axis,
                    max_penetration=0.004,
                    positive_elem=f"{kind}_arbor",
                    negative_elem=f"{face}_dial",
                    name=f"{face} {kind} arbor has shallow dial insertion",
                )
            ctx.expect_overlap(
                hand,
                tower,
                axes="z",
                min_overlap=0.20,
                elem_a=f"{kind}_blade",
                elem_b=f"{face}_dial",
                name=f"{face} {kind} hand lies over its clock face",
            )
        ctx.allow_overlap(
            object_model.get_part(f"{face}_minute_hand"),
            object_model.get_part(f"{face}_hour_hand"),
            elem_a="minute_arbor",
            elem_b="hour_boss",
            reason="Coaxial clock arbors are intentionally nested at the shared pivot.",
        )
        ctx.allow_overlap(
            object_model.get_part(f"{face}_minute_hand"),
            object_model.get_part(f"{face}_hour_hand"),
            elem_a="minute_arbor",
            elem_b="hour_arbor",
            reason="The concentric minute and hour arbors intentionally share the clock centerline.",
        )
        ctx.expect_overlap(
            object_model.get_part(f"{face}_minute_hand"),
            object_model.get_part(f"{face}_hour_hand"),
            axes="z",
            min_overlap=0.02,
            elem_a="minute_arbor",
            elem_b="hour_boss",
            name=f"{face} coaxial arbors share the pivot stack",
        )
        ctx.expect_overlap(
            object_model.get_part(f"{face}_minute_hand"),
            object_model.get_part(f"{face}_hour_hand"),
            axes="z",
            min_overlap=0.02,
            elem_a="minute_arbor",
            elem_b="hour_arbor",
            name=f"{face} concentric arbors share the centerline",
        )

    front_minute = object_model.get_part("front_minute_hand")
    front_minute_pivot = object_model.get_articulation("front_minute_pivot")
    rest_aabb = ctx.part_element_world_aabb(front_minute, elem="minute_blade")
    with ctx.pose({front_minute_pivot: math.pi / 2.0}):
        quarter_aabb = ctx.part_element_world_aabb(front_minute, elem="minute_blade")
    ctx.check(
        "front minute hand visibly rotates about its center pivot",
        rest_aabb is not None
        and quarter_aabb is not None
        and quarter_aabb[1][2] < rest_aabb[1][2] - 0.20,
        details=f"rest_aabb={rest_aabb}, quarter_aabb={quarter_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
