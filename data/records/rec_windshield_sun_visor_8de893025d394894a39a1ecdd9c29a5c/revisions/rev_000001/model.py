from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


VISOR_LEN = 0.318
VISOR_DEPTH = 0.160
VISOR_THICK = 0.018
VISOR_TOP_Z = -0.003
VISOR_BOTTOM_Z = VISOR_TOP_Z - VISOR_THICK
VISOR_START_X = 0.012
VISOR_END_X = VISOR_START_X + VISOR_LEN
VISOR_CORNER_R = 0.022

MIRROR_OPENING_W = 0.176
MIRROR_OPENING_H = 0.078
MIRROR_RECESS_D = 0.005
MIRROR_COVER_W = 0.168
MIRROR_COVER_H = 0.070
MIRROR_COVER_T = 0.0035
MIRROR_CENTER_X = 0.178
MIRROR_CENTER_Y = 0.088
MIRROR_POCKET_TOP = MIRROR_CENTER_Y - (MIRROR_OPENING_H / 2.0)
MIRROR_FACE_Z = VISOR_BOTTOM_Z + 0.0005

SWIVEL_TO_VISOR = (0.018, 0.0, -0.015)
RETAINER_X = VISOR_END_X - 0.008
RETAINER_Y = 0.048
ROOF_STRIP_L = 0.372
ROOF_STRIP_W = 0.120
ROOF_STRIP_T = 0.010
ROOF_BOTTOM_Z = 0.011
ROOF_TOP_Z = ROOF_BOTTOM_Z + ROOF_STRIP_T


def _filleted_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def _build_mount_shape() -> cq.Workplane:
    roof = _filleted_box((ROOF_STRIP_L, ROOF_STRIP_W, ROOF_STRIP_T), 0.003).translate(
        (0.165, 0.030, ROOF_BOTTOM_Z + (ROOF_STRIP_T / 2.0))
    )
    bracket = _filleted_box((0.050, 0.046, 0.016), 0.006).translate((0.010, 0.0, 0.011))
    socket = cq.Workplane("XY").cylinder(0.008, 0.009).translate((0.0, 0.0, 0.004))
    return roof.union(bracket).union(socket)


def _build_swivel_shape() -> cq.Workplane:
    stem = cq.Workplane("XY").cylinder(0.018, 0.0058).translate((0.0, 0.0, -0.009))
    knuckle = _filleted_box((0.024, 0.024, 0.014), 0.004).translate((0.010, 0.0, -0.016))
    arm = _filleted_box((0.018, 0.014, 0.010), 0.003).translate((0.020, 0.0, -0.015))
    contact_pad = cq.Workplane("XY").box(0.002, 0.010, 0.008).translate((0.026, 0.0, -0.015))
    return stem.union(knuckle).union(arm).union(contact_pad)


def _build_visor_shape() -> cq.Workplane:
    body_center = (
        VISOR_START_X + (VISOR_LEN / 2.0),
        VISOR_DEPTH / 2.0,
        VISOR_TOP_Z - (VISOR_THICK / 2.0),
    )
    body = _filleted_box((VISOR_LEN, VISOR_DEPTH, VISOR_THICK), VISOR_CORNER_R).translate(body_center)

    pocket = _filleted_box(
        (MIRROR_OPENING_W, MIRROR_OPENING_H, MIRROR_RECESS_D + 0.001),
        0.010,
    ).translate(
        (
            MIRROR_CENTER_X,
            MIRROR_CENTER_Y,
            VISOR_BOTTOM_Z + ((MIRROR_RECESS_D + 0.001) / 2.0),
        )
    )

    catch = _filleted_box((0.020, 0.014, 0.004), 0.002).translate(
        (VISOR_END_X - 0.010, RETAINER_Y, VISOR_TOP_Z + 0.002)
    )
    hinge_boss = _filleted_box((0.010, 0.016, 0.008), 0.0025).translate((0.016, 0.000, -0.013))
    hinge_contact = cq.Workplane("XY").box(0.006, 0.010, 0.008).translate((0.008, 0.000, -0.015))

    return body.cut(pocket).union(catch).union(hinge_boss).union(hinge_contact)


def _build_mirror_cover_shape() -> cq.Workplane:
    cover = _filleted_box((MIRROR_COVER_W, MIRROR_COVER_H, MIRROR_COVER_T), 0.008).translate(
        (0.0, MIRROR_COVER_H / 2.0, MIRROR_COVER_T / 2.0)
    )
    hinge_rail = _filleted_box((0.110, 0.004, 0.0020), 0.0008).translate((0.0, 0.002, 0.0018))
    finger_lip = _filleted_box((0.046, 0.010, 0.0022), 0.0018).translate(
        (0.0, MIRROR_COVER_H - 0.006, -0.0008)
    )
    return cover.union(hinge_rail).union(finger_lip)


def _build_retainer_clip_shape() -> cq.Workplane:
    pad = _filleted_box((0.022, 0.022, 0.002), 0.001).translate((0.0, 0.0, -0.001))
    jaw_a = _filleted_box((0.004, 0.016, 0.012), 0.0012).translate((-0.005, 0.0, -0.008))
    jaw_b = _filleted_box((0.004, 0.016, 0.012), 0.0012).translate((0.005, 0.0, -0.008))
    latch = _filleted_box((0.014, 0.004, 0.002), 0.0008).translate((0.0, 0.004, -0.013))
    hanger = _filleted_box((0.006, 0.010, ROOF_BOTTOM_Z), 0.0008).translate((0.0, 0.0, ROOF_BOTTOM_Z / 2.0))
    return pad.union(jaw_a).union(jaw_b).union(latch).union(hanger)


def _build_ticket_clip_shape() -> cq.Workplane:
    pad = _filleted_box((0.0022, 0.024, 0.010), 0.0008).translate((-0.0011, 0.0, -0.005))
    tongue = _filleted_box((0.026, 0.018, 0.0016), 0.0008).translate((0.013, 0.0, -0.0032))
    toe = _filleted_box((0.006, 0.018, 0.0016), 0.0008).translate((0.026, 0.0, -0.0016))
    return pad.union(tongue).union(toe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_visor")

    model.material("headliner", rgba=(0.82, 0.82, 0.80, 1.0))
    model.material("hardware", rgba=(0.34, 0.35, 0.38, 1.0))
    model.material("visor_fabric", rgba=(0.74, 0.72, 0.66, 1.0))
    model.material("cover_skin", rgba=(0.77, 0.75, 0.69, 1.0))
    model.material("clip_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    mount = model.part("mount")
    mount.visual(
        mesh_from_cadquery(_build_mount_shape(), "mount"),
        material="headliner",
        name="mount_shell",
    )

    swivel = model.part("swivel")
    swivel.visual(
        mesh_from_cadquery(_build_swivel_shape(), "swivel"),
        material="hardware",
        name="swivel_body",
    )

    visor = model.part("visor")
    visor.visual(
        mesh_from_cadquery(_build_visor_shape(), "visor"),
        material="visor_fabric",
        name="visor_body",
    )

    mirror_cover = model.part("mirror_cover")
    mirror_cover.visual(
        mesh_from_cadquery(_build_mirror_cover_shape(), "mirror_cover"),
        material="cover_skin",
        name="mirror_cover",
    )

    retainer_clip = model.part("retainer_clip")
    retainer_clip.visual(
        mesh_from_cadquery(_build_retainer_clip_shape(), "retainer_clip"),
        material="clip_dark",
        name="retainer_clip",
    )

    ticket_clip = model.part("ticket_clip")
    ticket_clip.visual(
        mesh_from_cadquery(_build_ticket_clip_shape(), "ticket_clip"),
        material="clip_dark",
        name="ticket_clip",
    )

    mount_swivel = model.articulation(
        "mount_swivel",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=swivel,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=8.0, velocity=1.8),
    )
    visor_hinge = model.articulation(
        "visor_hinge",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=visor,
        origin=Origin(xyz=SWIVEL_TO_VISOR),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=8.0, velocity=1.8),
    )
    mirror_hinge = model.articulation(
        "mirror_hinge",
        ArticulationType.REVOLUTE,
        parent=visor,
        child=mirror_cover,
        origin=Origin(xyz=(MIRROR_CENTER_X, MIRROR_POCKET_TOP, MIRROR_FACE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.15, effort=1.5, velocity=2.5),
    )
    model.articulation(
        "retainer_mount",
        ArticulationType.FIXED,
        parent=mount,
        child=retainer_clip,
        origin=Origin(xyz=(RETAINER_X, RETAINER_Y, 0.0)),
    )
    model.articulation(
        "ticket_mount",
        ArticulationType.FIXED,
        parent=visor,
        child=ticket_clip,
        origin=Origin(xyz=(VISOR_START_X, 0.056, VISOR_BOTTOM_Z)),
    )

    mount_swivel.meta = {"qc_samples": [0.0, 1.10]}
    visor_hinge.meta = {"qc_samples": [0.0, 1.20]}
    mirror_hinge.meta = {"qc_samples": [0.0, 1.60]}

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    visor = object_model.get_part("visor")
    mirror_cover = object_model.get_part("mirror_cover")
    retainer_clip = object_model.get_part("retainer_clip")
    ticket_clip = object_model.get_part("ticket_clip")
    mount_swivel = object_model.get_articulation("mount_swivel")
    visor_hinge = object_model.get_articulation("visor_hinge")
    mirror_hinge = object_model.get_articulation("mirror_hinge")

    ctx.allow_overlap(
        "swivel",
        "visor",
        elem_a="swivel_body",
        elem_b="visor_body",
        reason="A concealed hinge lug nests inside the swivel knuckle to visually support the visor at the bracket.",
    )
    ctx.allow_overlap(
        "mount",
        "retainer_clip",
        elem_a="mount_shell",
        elem_b="retainer_clip",
        reason="The retainer hanger is intentionally simplified as embedded into the headliner mounting pad proxy.",
    )
    ctx.expect_contact(
        retainer_clip,
        mount,
        contact_tol=0.0015,
        name="retainer clip mounts to roof strip",
    )
    ctx.expect_contact(
        ticket_clip,
        visor,
        contact_tol=0.0015,
        name="ticket clip mounts to visor edge",
    )

    with ctx.pose({mirror_hinge: 0.0}):
        ctx.expect_within(
            mirror_cover,
            visor,
            axes="xy",
            margin=0.010,
            name="mirror cover stays inset within visor face",
        )
        ctx.expect_overlap(
            mirror_cover,
            visor,
            axes="xy",
            min_overlap=0.060,
            name="mirror cover sits in the visor panel opening",
        )

    closed_visor_aabb = None
    open_visor_aabb = None
    with ctx.pose({mount_swivel: 0.0, visor_hinge: 0.0}):
        closed_visor_aabb = ctx.part_world_aabb(visor)
    with ctx.pose({mount_swivel: 0.0, visor_hinge: 1.20}):
        open_visor_aabb = ctx.part_world_aabb(visor)

    ctx.check(
        "visor rotates downward from the roof hinge",
        closed_visor_aabb is not None
        and open_visor_aabb is not None
        and open_visor_aabb[0][2] < closed_visor_aabb[0][2] - 0.070,
        details=f"closed={closed_visor_aabb}, open={open_visor_aabb}",
    )

    rest_swivel_aabb = None
    side_swivel_aabb = None
    with ctx.pose({mount_swivel: 0.0, visor_hinge: 0.0}):
        rest_swivel_aabb = ctx.part_world_aabb(visor)
    with ctx.pose({mount_swivel: 1.10, visor_hinge: 0.0}):
        side_swivel_aabb = ctx.part_world_aabb(visor)

    ctx.check(
        "visor swivels sideways near the bracket",
        rest_swivel_aabb is not None
        and side_swivel_aabb is not None
        and side_swivel_aabb[1][1] > rest_swivel_aabb[1][1] + 0.090,
        details=f"rest={rest_swivel_aabb}, side={side_swivel_aabb}",
    )

    closed_cover_aabb = None
    open_cover_aabb = None
    with ctx.pose({mirror_hinge: 0.0}):
        closed_cover_aabb = ctx.part_world_aabb(mirror_cover)
    with ctx.pose({mirror_hinge: 1.60}):
        open_cover_aabb = ctx.part_world_aabb(mirror_cover)

    ctx.check(
        "mirror cover opens as a real flap",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][2] < closed_cover_aabb[0][2] - 0.030,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
