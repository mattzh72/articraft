from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FaceFrame = dict[str, object]


def _vadd(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vscale(a: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (a[0] * s, a[1] * s, a[2] * s)


def _local_point(
    face: FaceFrame, x: float = 0.0, y: float = 0.0, z: float = 0.0
) -> tuple[float, float, float]:
    center = face["center"]
    x_axis = face["x_axis"]
    y_axis = face["y_axis"]
    normal = face["normal"]
    return _vadd(
        _vadd(_vadd(center, _vscale(x_axis, x)), _vscale(y_axis, y)),
        _vscale(normal, z),
    )


def _arch_plate(width: float, height: float, thickness: float, name: str):
    """Pointed Gothic lancet plate, extruded along local +Z."""
    shoulder = height * 0.70
    profile = [
        (-width * 0.5, 0.0),
        (-width * 0.5, shoulder),
        (0.0, height),
        (width * 0.5, shoulder),
        (width * 0.5, 0.0),
    ]
    shape = cq.Workplane("XY").polyline(profile).close().extrude(thickness)
    return mesh_from_cadquery(shape, name, tolerance=0.004, angular_tolerance=0.12)


def _spire_mesh():
    spire = (
        cq.Workplane("XY")
        .rect(3.22, 3.22)
        .workplane(offset=2.22)
        .rect(0.26, 0.26)
        .loft()
    )
    return mesh_from_cadquery(spire, "slate_spire", tolerance=0.006, angular_tolerance=0.14)


def _bell_mesh():
    """Flared bronze bell body lofted upward from a broad mouth to a crown."""
    bell = (
        cq.Workplane("XY")
        .circle(0.34)
        .workplane(offset=0.08)
        .circle(0.31)
        .workplane(offset=0.23)
        .circle(0.21)
        .workplane(offset=0.23)
        .circle(0.10)
        .loft()
    )
    return mesh_from_cadquery(bell, "bronze_bell", tolerance=0.003, angular_tolerance=0.10)


def _add_box_on_face(
    part,
    face: FaceFrame,
    *,
    name: str,
    size: tuple[float, float, float],
    local_xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=_local_point(face, *local_xyz), rpy=face["rpy"]),
        material=material,
        name=name,
    )


def _add_arch_on_face(
    part,
    face: FaceFrame,
    *,
    mesh,
    name: str,
    width: float,
    height: float,
    center_y: float,
    local_z: float,
    material,
) -> None:
    # The mesh origin is its lower-center in face-local XY.
    part.visual(
        mesh,
        origin=Origin(
            xyz=_local_point(face, 0.0, center_y - height * 0.5, local_z),
            rpy=face["rpy"],
        ),
        material=material,
        name=name,
    )


def _add_clock_face(part, face: FaceFrame, label: str, materials: dict[str, object]) -> None:
    disk_radius = 0.62
    part.visual(
        Cylinder(radius=disk_radius, length=0.060),
        origin=Origin(xyz=_local_point(face, 0.0, 0.0, 0.025), rpy=face["rpy"]),
        material=materials["clock_face"],
        name=f"{label}_clock_face",
    )
    part.visual(
        Cylinder(radius=disk_radius + 0.055, length=0.070),
        origin=Origin(xyz=_local_point(face, 0.0, 0.0, 0.019), rpy=face["rpy"]),
        material=materials["dark_stone"],
        name=f"{label}_clock_bezel",
    )
    part.visual(
        Cylinder(radius=0.085, length=0.025),
        origin=Origin(xyz=_local_point(face, 0.0, 0.0, 0.067), rpy=face["rpy"]),
        material=materials["black"],
        name=f"{label}_clock_spindle",
    )
    for i in range(12):
        angle = i * math.tau / 12.0
        radius = 0.49
        x = math.sin(angle) * radius
        y = math.cos(angle) * radius
        marker_radius = 0.038 if i % 3 == 0 else 0.022
        part.visual(
            Cylinder(radius=marker_radius, length=0.012),
            origin=Origin(xyz=_local_point(face, x, y, 0.057), rpy=face["rpy"]),
            material=materials["black"],
            name=f"{label}_marker_{i}",
        )


def _add_course_bands(part, *, prefix: str, half: float, z_values: list[float], material) -> None:
    for index, z in enumerate(z_values):
        part.visual(
            Box((half * 2.0 + 0.06, 0.026, 0.028)),
            origin=Origin(xyz=(0.0, -half - 0.010, z)),
            material=material,
            name=f"{prefix}_front_course_{index}",
        )
        part.visual(
            Box((half * 2.0 + 0.06, 0.026, 0.028)),
            origin=Origin(xyz=(0.0, half + 0.010, z)),
            material=material,
            name=f"{prefix}_rear_course_{index}",
        )
        part.visual(
            Box((0.026, half * 2.0 + 0.06, 0.028)),
            origin=Origin(xyz=(half + 0.010, 0.0, z)),
            material=material,
            name=f"{prefix}_side_course_{index}_0",
        )
        part.visual(
            Box((0.026, half * 2.0 + 0.06, 0.028)),
            origin=Origin(xyz=(-half - 0.010, 0.0, z)),
            material=material,
            name=f"{prefix}_side_course_{index}_1",
        )


def _add_hand_visuals(
    part,
    *,
    tip: float,
    tail: float,
    width: float,
    thickness: float,
    z_center: float,
    angle: float,
    material,
    hub_radius: float,
    hub_name: str,
) -> None:
    length = tip + tail
    center_distance = (tip - tail) * 0.5
    cx = -math.sin(angle) * center_distance
    cy = math.cos(angle) * center_distance
    part.visual(
        Box((width, length, thickness)),
        origin=Origin(xyz=(cx, cy, z_center), rpy=(0.0, 0.0, angle)),
        material=material,
        name="pointer",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=thickness),
        origin=Origin(xyz=(0.0, 0.0, z_center)),
        material=material,
        name=hub_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gothic_clock_tower")

    stone = model.material("weathered_limestone", rgba=(0.57, 0.56, 0.52, 1.0))
    dark_stone = model.material("shadowed_stone", rgba=(0.30, 0.30, 0.28, 1.0))
    mortar = model.material("dark_mortar", rgba=(0.22, 0.22, 0.21, 1.0))
    clock_face = model.material("aged_clock_face", rgba=(0.88, 0.82, 0.63, 1.0))
    black = model.material("black_iron", rgba=(0.02, 0.02, 0.018, 1.0))
    slate = model.material("blue_black_slate", rgba=(0.08, 0.10, 0.13, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.58, 0.36, 0.12, 1.0))
    shadow = model.material("deep_arch_shadow", rgba=(0.015, 0.014, 0.013, 1.0))

    materials = {
        "stone": stone,
        "dark_stone": dark_stone,
        "mortar": mortar,
        "clock_face": clock_face,
        "black": black,
        "slate": slate,
        "bronze": bronze,
        "shadow": shadow,
    }

    narrow_arch = _arch_plate(0.42, 1.20, 0.035, "narrow_lancet_shadow")
    belfry_arch = _arch_plate(1.02, 1.52, 0.040, "belfry_lancet_shadow")
    spire_mesh = _spire_mesh()
    bell_mesh = _bell_mesh()

    tower = model.part("tower")

    # Square masonry shaft and staged tower massing.
    tower.visual(Box((4.25, 4.25, 0.42)), origin=Origin(xyz=(0.0, 0.0, 0.21)), material=stone, name="plinth")
    tower.visual(Box((3.45, 3.45, 0.34)), origin=Origin(xyz=(0.0, 0.0, 0.58)), material=stone, name="base_step")
    tower.visual(Box((2.86, 2.86, 9.80)), origin=Origin(xyz=(0.0, 0.0, 5.35)), material=stone, name="lower_shaft")
    tower.visual(Box((3.22, 3.22, 4.55)), origin=Origin(xyz=(0.0, 0.0, 12.05)), material=stone, name="clock_stage")
    tower.visual(Box((3.02, 3.02, 2.95)), origin=Origin(xyz=(0.0, 0.0, 15.60)), material=stone, name="belfry_stage")

    # Four stepped corner buttresses rooted into the plinth.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((0.44, 0.44, 9.95)),
                origin=Origin(xyz=(sx * 1.43, sy * 1.43, 5.55)),
                material=stone,
                name=f"buttress_{sx}_{sy}",
            )
            tower.visual(
                Box((0.58, 0.58, 0.34)),
                origin=Origin(xyz=(sx * 1.43, sy * 1.43, 1.04)),
                material=dark_stone,
                name=f"buttress_foot_{sx}_{sy}",
            )

    _add_course_bands(
        tower,
        prefix="shaft",
        half=1.43,
        z_values=[1.60, 2.55, 3.50, 4.45, 5.40, 6.35, 7.30, 8.25, 9.20],
        material=mortar,
    )
    _add_course_bands(
        tower,
        prefix="clock",
        half=1.61,
        z_values=[10.25, 11.35, 12.75, 13.75],
        material=mortar,
    )

    # Crenellated parapet and steep slate spire.
    tower.visual(Box((3.55, 3.55, 0.30)), origin=Origin(xyz=(0.0, 0.0, 17.02)), material=dark_stone, name="parapet_slab")
    tower.visual(Box((3.55, 0.22, 0.62)), origin=Origin(xyz=(0.0, -1.78, 17.34)), material=stone, name="front_parapet")
    tower.visual(Box((3.55, 0.22, 0.62)), origin=Origin(xyz=(0.0, 1.78, 17.34)), material=stone, name="rear_parapet")
    tower.visual(Box((0.22, 3.55, 0.62)), origin=Origin(xyz=(1.78, 0.0, 17.34)), material=stone, name="side_parapet_0")
    tower.visual(Box((0.22, 3.55, 0.62)), origin=Origin(xyz=(-1.78, 0.0, 17.34)), material=stone, name="side_parapet_1")
    for i, pos in enumerate([-1.30, -0.43, 0.43, 1.30]):
        tower.visual(Box((0.42, 0.34, 0.72)), origin=Origin(xyz=(pos, -1.86, 17.55)), material=stone, name=f"crenel_front_{i}")
        tower.visual(Box((0.42, 0.34, 0.72)), origin=Origin(xyz=(pos, 1.86, 17.55)), material=stone, name=f"crenel_rear_{i}")
        tower.visual(Box((0.34, 0.42, 0.72)), origin=Origin(xyz=(1.86, pos, 17.55)), material=stone, name=f"crenel_side_{i}_0")
        tower.visual(Box((0.34, 0.42, 0.72)), origin=Origin(xyz=(-1.86, pos, 17.55)), material=stone, name=f"crenel_side_{i}_1")
    tower.visual(spire_mesh, origin=Origin(xyz=(0.0, 0.0, 16.96)), material=slate, name="slate_spire")
    tower.visual(Cylinder(radius=0.045, length=0.92), origin=Origin(xyz=(0.0, 0.0, 19.58)), material=black, name="finial_spike")
    tower.visual(Box((0.55, 0.055, 0.055)), origin=Origin(xyz=(0.0, 0.0, 19.83)), material=black, name="finial_cross")

    face_half = 1.61
    clock_z = 12.05
    faces: dict[str, FaceFrame] = {
        "front": {
            "center": (0.0, -face_half, clock_z),
            "x_axis": (1.0, 0.0, 0.0),
            "y_axis": (0.0, 0.0, 1.0),
            "normal": (0.0, -1.0, 0.0),
            "rpy": (math.pi / 2.0, 0.0, 0.0),
        },
        "rear": {
            "center": (0.0, face_half, clock_z),
            "x_axis": (-1.0, 0.0, 0.0),
            "y_axis": (0.0, 0.0, 1.0),
            "normal": (0.0, 1.0, 0.0),
            "rpy": (math.pi / 2.0, 0.0, math.pi),
        },
        "right": {
            "center": (face_half, 0.0, clock_z),
            "x_axis": (0.0, 1.0, 0.0),
            "y_axis": (0.0, 0.0, 1.0),
            "normal": (1.0, 0.0, 0.0),
            "rpy": (math.pi / 2.0, 0.0, math.pi / 2.0),
        },
        "left": {
            "center": (-face_half, 0.0, clock_z),
            "x_axis": (0.0, -1.0, 0.0),
            "y_axis": (0.0, 0.0, 1.0),
            "normal": (-1.0, 0.0, 0.0),
            "rpy": (math.pi / 2.0, 0.0, -math.pi / 2.0),
        },
    }

    # Gothic lancet windows below and bell louver openings above each clock.
    lower_half = 1.43
    lower_window_faces = {
        key: dict(value, center=_vscale(value["normal"], lower_half)[:2] + (6.05,))
        for key, value in faces.items()
    }
    belfry_faces = {
        key: dict(value, center=(value["center"][0] * 0.94, value["center"][1] * 0.94, 15.45))
        for key, value in faces.items()
    }
    for label, face in lower_window_faces.items():
        _add_arch_on_face(
            tower,
            face,
            mesh=narrow_arch,
            name=f"{label}_lower_lancet",
            width=0.42,
            height=1.20,
            center_y=0.0,
            local_z=-0.018,
            material=shadow,
        )
    for label, face in belfry_faces.items():
        _add_arch_on_face(
            tower,
            face,
            mesh=belfry_arch,
            name=f"{label}_belfry_arch",
            width=1.02,
            height=1.52,
            center_y=0.0,
            local_z=-0.018,
            material=shadow,
        )
        _add_box_on_face(
            tower,
            face,
            name=f"{label}_belfry_mullion",
            size=(0.060, 1.05, 0.065),
            local_xyz=(0.0, -0.12, 0.030),
            material=stone,
        )
        for j, y in enumerate([-0.34, -0.12, 0.10]):
            _add_box_on_face(
                tower,
                face,
                name=f"{label}_louver_{j}",
                size=(0.74, 0.045, 0.075),
                local_xyz=(0.0, y, 0.045),
                material=dark_stone,
            )

    # A visible bronze bell and yoke in the front belfry opening.
    tower.visual(Box((1.25, 0.16, 0.18)), origin=Origin(xyz=(0.0, -1.54, 15.66)), material=dark_stone, name="bell_yoke")
    tower.visual(Cylinder(radius=0.080, length=0.30), origin=Origin(xyz=(0.0, -1.61, 15.48)), material=bronze, name="bell_crown")
    tower.visual(bell_mesh, origin=Origin(xyz=(0.0, -1.72, 14.79)), material=bronze, name="bell")
    tower.visual(Cylinder(radius=0.018, length=0.48), origin=Origin(xyz=(0.0, -1.72, 14.98)), material=black, name="clapper_hanger")
    tower.visual(Sphere(radius=0.075), origin=Origin(xyz=(0.0, -1.72, 14.70)), material=black, name="bell_clapper")

    for label, face in faces.items():
        _add_clock_face(tower, face, label, materials)

    # Eight independent coaxial clock-hand parts: minute and hour hand on each face.
    minute_angle = -math.pi / 3.0  # ten minutes past
    hour_angle = math.pi / 3.0     # near ten o'clock, classic clock display
    hand_specs = [
        ("front", faces["front"]),
        ("rear", faces["rear"]),
        ("right", faces["right"]),
        ("left", faces["left"]),
    ]
    for label, face in hand_specs:
        hour = model.part(f"{label}_hour_hand")
        _add_hand_visuals(
            hour,
            tip=0.34,
            tail=0.065,
            width=0.070,
            thickness=0.012,
            z_center=0.006,
            angle=hour_angle,
            material=black,
            hub_radius=0.070,
            hub_name="hour_hub",
        )
        minute = model.part(f"{label}_minute_hand")
        _add_hand_visuals(
            minute,
            tip=0.52,
            tail=0.080,
            width=0.045,
            thickness=0.012,
            z_center=0.029,
            angle=minute_angle,
            material=black,
            hub_radius=0.050,
            hub_name="minute_cap",
        )
        minute.visual(
            Cylinder(radius=0.045, length=0.011),
            origin=Origin(xyz=(0.0, 0.0, 0.0175)),
            material=black,
            name="minute_spacer",
        )
        # The child frames sit at the outer face of the fixed clock spindle.
        # The hour hub touches the spindle; the minute spacer touches the hour hub,
        # so both coaxial hand parts are physically retained without floating.
        joint_origin = Origin(xyz=_local_point(face, 0.0, 0.0, 0.0795), rpy=face["rpy"])
        model.articulation(
            f"{label}_hour_axis",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=hour,
            origin=joint_origin,
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=0.08),
        )
        model.articulation(
            f"{label}_minute_axis",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=minute,
            origin=joint_origin,
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=1.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    face_axes = {
        "front": ("xz", "y", "tower_positive"),
        "rear": ("xz", "y", "hand_positive"),
        "right": ("yz", "x", "hand_positive"),
        "left": ("yz", "x", "tower_positive"),
    }

    continuous_names = []
    for label, (projection_axes, normal_axis, positive_side) in face_axes.items():
        hour = object_model.get_part(f"{label}_hour_hand")
        minute = object_model.get_part(f"{label}_minute_hand")
        hour_joint = object_model.get_articulation(f"{label}_hour_axis")
        minute_joint = object_model.get_articulation(f"{label}_minute_axis")
        continuous_names.extend([hour_joint.name, minute_joint.name])

        ctx.check(
            f"{label} hands use continuous coaxial joints",
            hour_joint.articulation_type == ArticulationType.CONTINUOUS
            and minute_joint.articulation_type == ArticulationType.CONTINUOUS
            and hour_joint.origin.xyz == minute_joint.origin.xyz
            and hour_joint.axis == minute_joint.axis,
            details=f"hour={hour_joint}, minute={minute_joint}",
        )
        ctx.expect_overlap(
            hour,
            tower,
            axes=projection_axes,
            elem_a="pointer",
            elem_b=f"{label}_clock_face",
            min_overlap=0.12,
            name=f"{label} hour hand lies over its dial",
        )
        ctx.expect_overlap(
            minute,
            tower,
            axes=projection_axes,
            elem_a="pointer",
            elem_b=f"{label}_clock_face",
            min_overlap=0.18,
            name=f"{label} minute hand lies over its dial",
        )
        if positive_side == "hand_positive":
            ctx.expect_gap(
                minute,
                hour,
                axis=normal_axis,
                min_gap=0.006,
                max_gap=0.020,
                positive_elem="pointer",
                negative_elem="pointer",
                name=f"{label} hand layers are separated",
            )
        else:
            ctx.expect_gap(
                hour,
                minute,
                axis=normal_axis,
                min_gap=0.006,
                max_gap=0.020,
                positive_elem="pointer",
                negative_elem="pointer",
                name=f"{label} hand layers are separated",
            )

    ctx.check(
        "four clock faces have two moving hands each",
        len(continuous_names) == 8,
        details=f"continuous hand joints={continuous_names}",
    )

    with ctx.pose({"front_minute_axis": math.pi / 2.0, "front_hour_axis": -math.pi / 4.0}):
        ctx.expect_overlap(
            object_model.get_part("front_minute_hand"),
            tower,
            axes="xz",
            elem_a="pointer",
            elem_b="front_clock_face",
            min_overlap=0.12,
            name="front minute hand stays on dial while rotating",
        )

    return ctx.report()


object_model = build_object_model()
