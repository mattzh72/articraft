from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CLOCK_Z = 9.35
CLOCK_RADIUS = 0.92
CLOCK_FACE_N = 0.095
CLOCK_HAND_HOUR_N = 0.225
CLOCK_HAND_MINUTE_N = 0.340


def _square_pyramid(width: float, depth: float, z_base: float, z_apex: float) -> MeshGeometry:
    """A simple square/rectangular civic-tower roof mesh in model coordinates."""
    geom = MeshGeometry()
    v0 = geom.add_vertex(-width / 2.0, -depth / 2.0, z_base)
    v1 = geom.add_vertex(width / 2.0, -depth / 2.0, z_base)
    v2 = geom.add_vertex(width / 2.0, depth / 2.0, z_base)
    v3 = geom.add_vertex(-width / 2.0, depth / 2.0, z_base)
    apex = geom.add_vertex(0.0, 0.0, z_apex)
    geom.add_face(v0, v1, apex)
    geom.add_face(v1, v2, apex)
    geom.add_face(v2, v3, apex)
    geom.add_face(v3, v0, apex)
    geom.add_face(v0, v2, v1)
    geom.add_face(v0, v3, v2)
    return geom


def _face_frame(face: str, half_width: float, half_depth: float) -> dict[str, object]:
    """Return a local face basis: local x is horizontal, local y is upward, z is outward."""
    if face == "front":
        return {
            "origin": (0.0, -half_depth, 0.0),
            "u": (1.0, 0.0, 0.0),
            "v": (0.0, 0.0, 1.0),
            "n": (0.0, -1.0, 0.0),
            "rpy": (pi / 2.0, 0.0, 0.0),
        }
    if face == "back":
        return {
            "origin": (0.0, half_depth, 0.0),
            "u": (-1.0, 0.0, 0.0),
            "v": (0.0, 0.0, 1.0),
            "n": (0.0, 1.0, 0.0),
            "rpy": (pi / 2.0, 0.0, pi),
        }
    if face == "east":
        return {
            "origin": (half_width, 0.0, 0.0),
            "u": (0.0, 1.0, 0.0),
            "v": (0.0, 0.0, 1.0),
            "n": (1.0, 0.0, 0.0),
            "rpy": (pi / 2.0, 0.0, pi / 2.0),
        }
    if face == "west":
        return {
            "origin": (-half_width, 0.0, 0.0),
            "u": (0.0, -1.0, 0.0),
            "v": (0.0, 0.0, 1.0),
            "n": (-1.0, 0.0, 0.0),
            "rpy": (pi / 2.0, 0.0, -pi / 2.0),
        }
    raise ValueError(f"Unknown face {face!r}")


def _face_point(frame: dict[str, object], u: float, v: float, n: float) -> tuple[float, float, float]:
    ox, oy, oz = frame["origin"]  # type: ignore[misc]
    ux, uy, uz = frame["u"]  # type: ignore[misc]
    vx, vy, vz = frame["v"]  # type: ignore[misc]
    nx, ny, nz = frame["n"]  # type: ignore[misc]
    return (
        ox + u * ux + v * vx + n * nx,
        oy + u * uy + v * vy + n * ny,
        oz + u * uz + v * vz + n * nz,
    )


def _add_face_box(
    part,
    frame: dict[str, object],
    *,
    u: float,
    v: float,
    n: float,
    size: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=_face_point(frame, u, v, n), rpy=frame["rpy"]),  # type: ignore[arg-type]
        material=material,
        name=name,
    )


def _add_face_cylinder(
    part,
    frame: dict[str, object],
    *,
    u: float,
    v: float,
    n: float,
    radius: float,
    length: float,
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_face_point(frame, u, v, n), rpy=frame["rpy"]),  # type: ignore[arg-type]
        material=material,
        name=name,
    )


def _add_window(part, frame: dict[str, object], *, u: float, z: float, name: str) -> None:
    """Inset-looking tall civic windows with stone trim and dark glazing."""
    _add_face_box(part, frame, u=u, v=z, n=0.020, size=(0.62, 0.96, 0.050), material="trim_stone", name=f"{name}_surround")
    _add_face_box(part, frame, u=u, v=z, n=0.036, size=(0.42, 0.74, 0.050), material="dark_glass", name=f"{name}_glass")
    _add_face_box(part, frame, u=u - 0.27, v=z, n=0.045, size=(0.055, 0.92, 0.070), material="ashlar_stone", name=f"{name}_left_jamb")
    _add_face_box(part, frame, u=u + 0.27, v=z, n=0.045, size=(0.055, 0.92, 0.070), material="ashlar_stone", name=f"{name}_right_jamb")
    _add_face_box(part, frame, u=u, v=z + 0.44, n=0.045, size=(0.62, 0.070, 0.070), material="ashlar_stone", name=f"{name}_lintel")
    _add_face_box(part, frame, u=u, v=z - 0.44, n=0.045, size=(0.70, 0.075, 0.072), material="trim_stone", name=f"{name}_sill")
    # Two muntins make each window read as a civic building element rather than a flat patch.
    _add_face_box(part, frame, u=u, v=z, n=0.066, size=(0.030, 0.70, 0.040), material="aged_bronze", name=f"{name}_center_muntin")
    _add_face_box(part, frame, u=u, v=z, n=0.068, size=(0.38, 0.026, 0.040), material="aged_bronze", name=f"{name}_cross_muntin")


def _add_louvers(part, frame: dict[str, object], *, u: float, z: float, name: str) -> None:
    _add_face_box(part, frame, u=u, v=z, n=0.024, size=(0.82, 0.62, 0.060), material="trim_stone", name=f"{name}_frame")
    _add_face_box(part, frame, u=u, v=z, n=0.048, size=(0.62, 0.40, 0.060), material="shadow_recess", name=f"{name}_shadow")
    for idx, dz in enumerate((-0.15, -0.05, 0.05, 0.15)):
        _add_face_box(
            part,
            frame,
            u=u,
            v=z + dz,
            n=0.080,
            size=(0.64, 0.035, 0.080),
            material="aged_bronze",
            name=f"{name}_slat_{idx}",
        )


def _add_clock_face(part, face: str, frame: dict[str, object]) -> None:
    _add_face_box(
        part,
        frame,
        u=0.0,
        v=CLOCK_Z,
        n=0.035,
        size=(2.22, 2.22, 0.070),
        material="trim_stone",
        name=f"{face}_clock_panel",
    )
    _add_face_cylinder(
        part,
        frame,
        u=0.0,
        v=CLOCK_Z,
        n=CLOCK_FACE_N,
        radius=CLOCK_RADIUS + 0.10,
        length=0.070,
        material="aged_bronze",
        name=f"{face}_outer_bezel",
    )
    _add_face_cylinder(
        part,
        frame,
        u=0.0,
        v=CLOCK_Z,
        n=CLOCK_FACE_N + 0.020,
        radius=CLOCK_RADIUS,
        length=0.055,
        material="warm_ivory",
        name=f"{face}_face_plate",
    )
    for idx in range(12):
        theta = 2.0 * pi * idx / 12.0
        radius = 0.045 if idx % 3 == 0 else 0.027
        _add_face_cylinder(
            part,
            frame,
            u=0.78 * sin(theta),
            v=CLOCK_Z + 0.78 * cos(theta),
            n=CLOCK_FACE_N + 0.055,
            radius=radius,
            length=0.032,
            material="charcoal_detail",
            name=f"{face}_hour_marker_{idx}",
        )


def _add_front_roman_numerals(part, frame: dict[str, object]) -> None:
    """Simple raised-bar Roman numerals at XII, III, VI, and IX on the front dial."""
    # XII
    _add_face_box(part, frame, u=-0.155, v=CLOCK_Z + 0.55, n=0.148, size=(0.030, 0.19, 0.022), material="charcoal_detail", name="front_roman_x_left")
    _add_face_box(part, frame, u=-0.105, v=CLOCK_Z + 0.55, n=0.148, size=(0.030, 0.19, 0.022), material="charcoal_detail", name="front_roman_x_right")
    _add_face_box(part, frame, u=-0.130, v=CLOCK_Z + 0.55, n=0.148, size=(0.115, 0.028, 0.024), material="charcoal_detail", name="front_roman_x_cross")
    _add_face_box(part, frame, u=0.020, v=CLOCK_Z + 0.55, n=0.148, size=(0.030, 0.19, 0.022), material="charcoal_detail", name="front_roman_i_0")
    _add_face_box(part, frame, u=0.085, v=CLOCK_Z + 0.55, n=0.148, size=(0.030, 0.19, 0.022), material="charcoal_detail", name="front_roman_i_1")
    # III
    for idx, du in enumerate((0.55, 0.61, 0.67)):
        _add_face_box(part, frame, u=du, v=CLOCK_Z, n=0.148, size=(0.028, 0.18, 0.022), material="charcoal_detail", name=f"front_roman_iii_{idx}")
    # VI
    _add_face_box(part, frame, u=-0.055, v=CLOCK_Z - 0.56, n=0.148, size=(0.115, 0.030, 0.022), material="charcoal_detail", name="front_roman_v_top")
    _add_face_box(part, frame, u=-0.083, v=CLOCK_Z - 0.58, n=0.148, size=(0.030, 0.15, 0.022), material="charcoal_detail", name="front_roman_v_left")
    _add_face_box(part, frame, u=-0.027, v=CLOCK_Z - 0.58, n=0.148, size=(0.030, 0.15, 0.022), material="charcoal_detail", name="front_roman_v_right")
    _add_face_box(part, frame, u=0.075, v=CLOCK_Z - 0.56, n=0.148, size=(0.030, 0.18, 0.022), material="charcoal_detail", name="front_roman_vi_i")
    # IX
    _add_face_box(part, frame, u=-0.64, v=CLOCK_Z, n=0.148, size=(0.028, 0.18, 0.022), material="charcoal_detail", name="front_roman_ix_i")
    _add_face_box(part, frame, u=-0.555, v=CLOCK_Z, n=0.148, size=(0.030, 0.18, 0.022), material="charcoal_detail", name="front_roman_ix_x_left")
    _add_face_box(part, frame, u=-0.505, v=CLOCK_Z, n=0.148, size=(0.030, 0.18, 0.022), material="charcoal_detail", name="front_roman_ix_x_right")
    _add_face_box(part, frame, u=-0.530, v=CLOCK_Z, n=0.148, size=(0.108, 0.028, 0.024), material="charcoal_detail", name="front_roman_ix_x_cross")


def _add_hand_geometry(
    part,
    *,
    length: float,
    width: float,
    material: str,
    prefix: str,
    arbor_radius: float,
    arbor_length: float,
    arbor_center_z: float,
    hub: bool = False,
) -> None:
    stem_radius = max(width * 0.46, arbor_radius)
    part.visual(
        Box((width, length, 0.026)),
        origin=Origin(xyz=(0.0, 0.035 + length / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_bar",
    )
    part.visual(
        Box((width * 0.74, 0.18, 0.024)),
        origin=Origin(xyz=(0.0, -0.035 - 0.09, 0.0)),
        material=material,
        name=f"{prefix}_counterweight",
    )
    part.visual(
        Sphere(radius=width * 0.78),
        origin=Origin(xyz=(0.0, 0.035 + length + width * 0.32, 0.0)),
        material=material,
        name=f"{prefix}_tip",
    )
    part.visual(
        Cylinder(radius=stem_radius, length=arbor_length),
        origin=Origin(xyz=(0.0, 0.0, arbor_center_z)),
        material="aged_bronze",
        name=f"{prefix}_arbor",
    )
    if hub:
        part.visual(
            Cylinder(radius=0.105, length=0.042),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="aged_bronze",
            name=f"{prefix}_hub",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civic_clock_tower")

    model.material("ashlar_stone", rgba=(0.62, 0.57, 0.49, 1.0))
    model.material("trim_stone", rgba=(0.80, 0.76, 0.66, 1.0))
    model.material("shadow_recess", rgba=(0.08, 0.075, 0.065, 1.0))
    model.material("dark_glass", rgba=(0.05, 0.08, 0.10, 0.78))
    model.material("faint_glass", rgba=(0.73, 0.84, 0.88, 0.28))
    model.material("aged_bronze", rgba=(0.49, 0.35, 0.15, 1.0))
    model.material("charcoal_detail", rgba=(0.035, 0.035, 0.032, 1.0))
    model.material("warm_ivory", rgba=(0.93, 0.88, 0.74, 1.0))
    model.material("slate_roof", rgba=(0.16, 0.22, 0.25, 1.0))
    model.material("hand_black", rgba=(0.02, 0.018, 0.016, 1.0))
    model.material("hand_gold", rgba=(0.83, 0.62, 0.18, 1.0))

    tower = model.part("tower")

    # Heavy civic base and main shaft.
    tower.visual(Box((5.40, 4.80, 0.26)), origin=Origin(xyz=(0.0, 0.0, 0.13)), material="trim_stone", name="lower_step")
    tower.visual(Box((4.85, 4.25, 0.28)), origin=Origin(xyz=(0.0, 0.0, 0.39)), material="ashlar_stone", name="upper_step")
    tower.visual(Box((4.20, 3.65, 0.48)), origin=Origin(xyz=(0.0, 0.0, 0.76)), material="trim_stone", name="plinth")
    tower.visual(Box((3.40, 2.90, 6.90)), origin=Origin(xyz=(0.0, 0.0, 4.12)), material="ashlar_stone", name="main_shaft")

    for idx, (x, y) in enumerate(((-1.62, -1.37), (1.62, -1.37), (-1.62, 1.37), (1.62, 1.37))):
        tower.visual(Box((0.34, 0.34, 6.98)), origin=Origin(xyz=(x, y, 4.15)), material="trim_stone", name=f"corner_pilaster_{idx}")

    for idx, z in enumerate((1.18, 2.65, 5.00, 7.48)):
        tower.visual(Box((3.95, 3.42, 0.24)), origin=Origin(xyz=(0.0, 0.0, z)), material="trim_stone", name=f"belt_course_{idx}")

    tower.visual(Box((4.28, 3.78, 0.34)), origin=Origin(xyz=(0.0, 0.0, 7.70)), material="trim_stone", name="clock_cornice_base")
    tower.visual(Box((3.02, 2.72, 2.88)), origin=Origin(xyz=(0.0, 0.0, 9.17)), material="ashlar_stone", name="clock_chamber")
    tower.visual(Box((3.70, 3.34, 0.34)), origin=Origin(xyz=(0.0, 0.0, 10.72)), material="trim_stone", name="upper_cornice")
    tower.visual(Box((3.18, 2.86, 0.56)), origin=Origin(xyz=(0.0, 0.0, 11.17)), material="ashlar_stone", name="parapet_block")

    for idx, (x, y) in enumerate(((-1.40, -1.25), (1.40, -1.25), (-1.40, 1.25), (1.40, 1.25))):
        tower.visual(Cylinder(radius=0.18, length=3.05), origin=Origin(xyz=(x, y, 9.18)), material="trim_stone", name=f"belfry_round_pier_{idx}")
        tower.visual(Cylinder(radius=0.21, length=0.18), origin=Origin(xyz=(x, y, 10.80)), material="aged_bronze", name=f"pier_cap_{idx}")

    tower.visual(mesh_from_geometry(_square_pyramid(3.08, 2.78, 11.43, 13.80), "slate_pyramid_roof"), material="slate_roof", name="slate_pyramid_roof")
    tower.visual(Cylinder(radius=0.070, length=0.95), origin=Origin(xyz=(0.0, 0.0, 14.12)), material="aged_bronze", name="spire_rod")
    tower.visual(Sphere(radius=0.16), origin=Origin(xyz=(0.0, 0.0, 14.66)), material="aged_bronze", name="spire_ball")

    lower_frames = {
        "front": _face_frame("front", 1.70, 1.45),
        "back": _face_frame("back", 1.70, 1.45),
        "east": _face_frame("east", 1.70, 1.45),
        "west": _face_frame("west", 1.70, 1.45),
    }
    for face, frame in lower_frames.items():
        for idx, (u, z) in enumerate(((-0.72, 2.00), (0.72, 2.00), (-0.72, 4.35), (0.72, 4.35))):
            _add_window(tower, frame, u=u, z=z, name=f"{face}_window_{idx}")
        _add_louvers(tower, frame, u=0.0, z=6.28, name=f"{face}_louver")

    clock_frames = {
        "front": _face_frame("front", 1.51, 1.36),
        "back": _face_frame("back", 1.51, 1.36),
        "east": _face_frame("east", 1.51, 1.36),
        "west": _face_frame("west", 1.51, 1.36),
    }
    for face, frame in clock_frames.items():
        _add_clock_face(tower, face, frame)
    _add_front_roman_numerals(tower, clock_frames["front"])

    for face, frame in clock_frames.items():
        hour = model.part(f"{face}_hour")
        _add_hand_geometry(
            hour,
            length=0.56,
            width=0.082,
            material="hand_black",
            prefix="hour",
            arbor_radius=0.030,
            arbor_length=0.180,
            arbor_center_z=-0.075,
        )

        minute = model.part(f"{face}_minute")
        _add_hand_geometry(
            minute,
            length=0.82,
            width=0.052,
            material="hand_gold",
            prefix="minute",
            arbor_radius=0.018,
            arbor_length=0.310,
            arbor_center_z=-0.150,
            hub=True,
        )

        hour_origin = Origin(xyz=_face_point(frame, 0.0, CLOCK_Z, CLOCK_HAND_HOUR_N), rpy=frame["rpy"])  # type: ignore[arg-type]
        minute_origin = Origin(xyz=_face_point(frame, 0.0, CLOCK_Z, CLOCK_HAND_MINUTE_N), rpy=frame["rpy"])  # type: ignore[arg-type]
        minute_joint_name = f"{face}_minute_spin"

        model.articulation(
            minute_joint_name,
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=minute,
            origin=minute_origin,
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.18, velocity=0.18),
        )
        model.articulation(
            f"{face}_hour_spin",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=hour,
            origin=hour_origin,
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.12, velocity=0.015),
            mimic=Mimic(joint=minute_joint_name, multiplier=1.0 / 12.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    front_minute = object_model.get_part("front_minute")
    front_hour = object_model.get_part("front_hour")
    minute_joint = object_model.get_articulation("front_minute_spin")
    hour_joint = object_model.get_articulation("front_hour_spin")

    for face in ("front", "back", "east", "west"):
        hour = object_model.get_part(f"{face}_hour")
        minute = object_model.get_part(f"{face}_minute")
        hour_arbor = hour.get_visual("hour_arbor")
        minute_arbor = minute.get_visual("minute_arbor")
        for elem in (f"{face}_clock_panel", f"{face}_outer_bezel", f"{face}_face_plate"):
            dial_elem = tower.get_visual(elem)
            ctx.allow_overlap(
                hour,
                tower,
                elem_a=hour_arbor,
                elem_b=dial_elem,
                reason="The hour hand arbor is intentionally captured through the clock dial hub.",
            )
            ctx.expect_overlap(
                hour,
                tower,
                axes="xyz",
                elem_a=hour_arbor,
                elem_b=dial_elem,
                min_overlap=0.010,
                name=f"{face} hour arbor is captured in {elem}",
            )
            ctx.allow_overlap(
                minute,
                tower,
                elem_a=minute_arbor,
                elem_b=dial_elem,
                reason="The minute hand arbor is intentionally captured through the clock dial hub.",
            )
            ctx.expect_overlap(
                minute,
                tower,
                axes="xyz",
                elem_a=minute_arbor,
                elem_b=dial_elem,
                min_overlap=0.010,
                name=f"{face} minute arbor is captured in {elem}",
            )
        ctx.allow_overlap(
            hour,
            minute,
            elem_a=hour_arbor,
            elem_b=minute_arbor,
            reason="The coaxial hour and minute arbors share the real clock-hand spindle line.",
        )
        ctx.expect_overlap(
            hour,
            minute,
            axes="xyz",
            elem_a=hour_arbor,
            elem_b=minute_arbor,
            min_overlap=0.010,
            name=f"{face} coaxial arbors share the spindle line",
        )

    ctx.check(
        "four large clock faces",
        all(tower.get_visual(f"{face}_face_plate") is not None for face in ("front", "back", "east", "west")),
        details="A civic tower should carry a clock face on each side of the belfry.",
    )
    ctx.check(
        "hour hand geared to minute hand",
        hour_joint.mimic is not None and abs(hour_joint.mimic.multiplier - (1.0 / 12.0)) < 1e-9,
        details=f"mimic={hour_joint.mimic!r}",
    )

    ctx.expect_overlap(
        front_minute,
        tower,
        axes="z",
        elem_a=front_minute.get_visual("minute_bar"),
        elem_b=tower.get_visual("front_face_plate"),
        min_overlap=0.72,
        name="front minute hand lies over the dial",
    )
    ctx.expect_overlap(
        front_hour,
        tower,
        axes="z",
        elem_a=front_hour.get_visual("hour_bar"),
        elem_b=tower.get_visual("front_face_plate"),
        min_overlap=0.48,
        name="front hour hand lies over the dial",
    )
    ctx.expect_gap(
        tower,
        front_hour,
        axis="y",
        positive_elem=tower.get_visual("front_face_plate"),
        negative_elem=front_hour.get_visual("hour_bar"),
        min_gap=0.020,
        name="front hour hand is proud of the face",
    )
    ctx.expect_gap(
        front_hour,
        front_minute,
        axis="y",
        positive_elem=front_hour.get_visual("hour_bar"),
        negative_elem=front_minute.get_visual("minute_bar"),
        min_gap=0.030,
        name="front minute hand is layered outside the hour hand",
    )

    rest_tip = ctx.part_element_world_aabb(front_minute, elem="minute_tip")
    with ctx.pose({minute_joint: pi / 2.0}):
        turned_tip = ctx.part_element_world_aabb(front_minute, elem="minute_tip")
    if rest_tip is not None and turned_tip is not None:
        rest_cx = (rest_tip[0][0] + rest_tip[1][0]) / 2.0
        turned_cx = (turned_tip[0][0] + turned_tip[1][0]) / 2.0
        turned_cz = (turned_tip[0][2] + turned_tip[1][2]) / 2.0
        ctx.check(
            "minute hand turns clockwise from twelve",
            abs(rest_cx) < 0.08 and turned_cx > 0.62 and abs(turned_cz - CLOCK_Z) < 0.10,
            details=f"rest_tip={rest_tip}, turned_tip={turned_tip}",
        )
    else:
        ctx.fail("minute hand tip is measurable", "Could not measure minute hand tip AABBs.")

    return ctx.report()


object_model = build_object_model()
