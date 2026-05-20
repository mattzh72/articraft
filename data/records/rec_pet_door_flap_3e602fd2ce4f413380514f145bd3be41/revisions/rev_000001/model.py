from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    BezelMounts,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_pet_door")

    wood = model.material("painted_door", rgba=(0.74, 0.62, 0.46, 1.0))
    plastic = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.77, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.60, 0.55, 1.0))
    smoky_poly = model.material("smoky_translucent_flap", rgba=(0.30, 0.42, 0.48, 0.46))
    magnet = model.material("magnetic_weight", rgba=(0.10, 0.10, 0.095, 1.0))

    door = model.part("door_panel")

    # A realistic cut-away section of a household door/wall.  It is built as
    # four connected rails around a true opening rather than a solid slab.
    door.visual(
        Box((0.86, 0.045, 0.200)),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=wood,
        name="upper_door_rail",
    )
    door.visual(
        Box((0.86, 0.045, 0.170)),
        origin=Origin(xyz=(0.0, 0.0, -0.335)),
        material=wood,
        name="lower_door_rail",
    )
    for x, name in ((-0.305, "side_door_stile_0"), (0.305, "side_door_stile_1")):
        door.visual(
            Box((0.250, 0.045, 0.530)),
            origin=Origin(xyz=(x, 0.0, 0.015)),
            material=wood,
            name=name,
        )

    # The front and rear consumer trim rings are rounded, screw-bossed bezels
    # mounted proud of the door surface.
    front_frame = BezelGeometry(
        (0.340, 0.520),
        (0.470, 0.660),
        0.028,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.045,
        outer_corner_radius=0.065,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
        mounts=BezelMounts(
            style="bosses",
            hole_count=4,
            hole_diameter=0.006,
            boss_diameter=0.026,
            setback=0.038,
        ),
    )
    door.visual(
        mesh_from_geometry(front_frame, "front_frame"),
        # Local bezel Z is depth.  Rotate it so the opening lies in world XZ
        # and the frame projects from the front face of the door.
        origin=Origin(xyz=(0.0, -0.0365, 0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="front_frame",
    )

    rear_frame = BezelGeometry(
        (0.350, 0.530),
        (0.445, 0.620),
        0.024,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.040,
        outer_corner_radius=0.055,
        face=BezelFace(style="radiused_step", front_lip=0.003, fillet=0.0015),
    )
    door.visual(
        mesh_from_geometry(rear_frame, "rear_frame"),
        origin=Origin(xyz=(0.0, 0.0345, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="rear_frame",
    )

    # Through-door tunnel liner, separate from the swinging flap clearance.
    door.visual(
        Box((0.012, 0.047, 0.520)),
        origin=Origin(xyz=(-0.174, 0.0, 0.015)),
        material=plastic,
        name="liner_side_0",
    )
    door.visual(
        Box((0.012, 0.047, 0.520)),
        origin=Origin(xyz=(0.174, 0.0, 0.015)),
        material=plastic,
        name="liner_side_1",
    )
    door.visual(
        Box((0.348, 0.047, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.281)),
        material=plastic,
        name="liner_top",
    )
    door.visual(
        Box((0.348, 0.047, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.251)),
        material=plastic,
        name="liner_bottom",
    )

    # Compressible looking weather seal on the front face, held just outside
    # the sweep envelope of the flap panel.
    door.visual(
        Box((0.012, 0.008, 0.455)),
        origin=Origin(xyz=(-0.166, -0.0525, 0.012)),
        material=dark_rubber,
        name="seal_side_0",
    )
    door.visual(
        Box((0.012, 0.008, 0.455)),
        origin=Origin(xyz=(0.166, -0.0525, 0.012)),
        material=dark_rubber,
        name="seal_side_1",
    )
    door.visual(
        Box((0.320, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.0525, 0.266)),
        material=dark_rubber,
        name="seal_top",
    )
    door.visual(
        Box((0.320, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.0525, -0.244)),
        material=dark_rubber,
        name="seal_bottom",
    )

    # Visible fasteners with screwdriver slots.
    for idx, (x, z, slot_angle) in enumerate(
        (
            (-0.185, 0.295, 0.0),
            (0.185, 0.295, 0.25),
            (-0.185, -0.265, -0.15),
            (0.185, -0.265, 0.1),
        )
    ):
        door.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, -0.053, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"screw_head_{idx}",
        )
        door.visual(
            Box((0.020, 0.002, 0.003)),
            origin=Origin(xyz=(x, -0.0565, z), rpy=(0.0, 0.0, slot_angle)),
            material=dark_rubber,
            name=f"screw_slot_{idx}",
        )

    # Hinge saddle blocks and end bushings mounted into the top of the front
    # frame.  The flap's own hinge tube sits between these bushings.
    for x, suffix in ((-0.166, "0"), (0.166, "1")):
        door.visual(
            Box((0.034, 0.026, 0.052)),
            origin=Origin(xyz=(x, -0.025, 0.255)),
            material=plastic,
            name=f"hinge_block_{suffix}",
        )
    for x, suffix in ((-0.153, "0"), (0.153, "1")):
        door.visual(
            Cylinder(radius=0.014, length=0.026),
            origin=Origin(xyz=(x, -0.008, 0.235), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=plastic,
            name=f"hinge_bushing_{suffix}",
        )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.012, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="hinge_tube",
    )
    flap.visual(
        Box((0.285, 0.008, 0.405)),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=smoky_poly,
        name="clear_panel",
    )
    flap.visual(
        Box((0.260, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.003, -0.082)),
        material=smoky_poly,
        name="stiffener_rib_0",
    )
    flap.visual(
        Box((0.006, 0.010, 0.330)),
        origin=Origin(xyz=(-0.085, -0.003, -0.215)),
        material=smoky_poly,
        name="stiffener_rib_1",
    )
    flap.visual(
        Box((0.006, 0.010, 0.330)),
        origin=Origin(xyz=(0.085, -0.003, -0.215)),
        material=smoky_poly,
        name="stiffener_rib_2",
    )
    for x, suffix in ((-0.142, "0"), (0.142, "1")):
        flap.visual(
            Box((0.010, 0.010, 0.370)),
            origin=Origin(xyz=(x, 0.001, -0.215)),
            material=dark_rubber,
            name=f"flap_edge_{suffix}",
        )
    flap.visual(
        Box((0.270, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, 0.001, -0.404)),
        material=magnet,
        name="bottom_magnet",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=flap,
        origin=Origin(xyz=(0.0, -0.008, 0.235)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.10, upper=1.10),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("door_panel")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")

    limits = hinge.motion_limits
    ctx.check(
        "flap swings both directions",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -1.0
        and limits.upper >= 1.0,
        details=f"limits={limits}",
    )

    ctx.expect_overlap(
        flap,
        door,
        axes="xz",
        min_overlap=0.25,
        elem_a="clear_panel",
        elem_b="front_frame",
        name="closed flap fills framed opening",
    )

    def _coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    def _center_y(aabb) -> float | None:
        if aabb is None:
            return None
        return (_coord(aabb[0], 1) + _coord(aabb[1], 1)) * 0.5

    with ctx.pose({hinge: 0.0}):
        rest_bottom = _center_y(ctx.part_element_world_aabb(flap, elem="bottom_magnet"))
    with ctx.pose({hinge: 0.95}):
        inward_bottom = _center_y(ctx.part_element_world_aabb(flap, elem="bottom_magnet"))
    with ctx.pose({hinge: -0.95}):
        outward_bottom = _center_y(ctx.part_element_world_aabb(flap, elem="bottom_magnet"))

    ctx.check(
        "positive swing opens inward",
        rest_bottom is not None
        and inward_bottom is not None
        and inward_bottom > rest_bottom + 0.22,
        details=f"rest={rest_bottom}, inward={inward_bottom}",
    )
    ctx.check(
        "negative swing opens outward",
        rest_bottom is not None
        and outward_bottom is not None
        and outward_bottom < rest_bottom - 0.22,
        details=f"rest={rest_bottom}, outward={outward_bottom}",
    )

    return ctx.report()


object_model = build_object_model()
